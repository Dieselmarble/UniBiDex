#!/usr/bin/env python3
"""
Parallel playback dual-arm teleop data:
- Sequentially read left and right arm joint commands for each frame
- Optionally display teleoperation images
- Dual-arm control initialization & all controls can be parallel
- --mode=real Drive real robot; --mode=print Print only
"""
import argparse
import time
import logging
import sys
import zarr
import numpy as np
import yaml
import cv2
from unibidex_controller import SingleUniBiDexController
from concurrent.futures import ThreadPoolExecutor, as_completed

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

def main():
    parser = argparse.ArgumentParser(
        description="Playback dual-arm teleop data and drive in parallel (single process + thread pool)"
    )
    parser.add_argument(
        "--zarr_path", type=str, default="./teleop_data.zarr",
        help="Zarr data directory"
    )
    parser.add_argument(
        "--config", type=str,
        help="Bimanual configuration file (YAML), required only in real mode"
    )
    parser.add_argument(
        "--mode", choices=["real", "print"], default="print",
        help="Mode: 'real' Drive real robot; 'print' Print only"
    )
    parser.add_argument(
        "--show-images", action="store_true",
        help="Whether to display teleoperation images"
    )
    args = parser.parse_args()
    mode = args.mode
    show_images = args.show_images

    # Open Zarr storage
    root = zarr.open_group(args.zarr_path, mode='r')
    cmds    = root['commands']       # (N,2,D)
    ts      = root['timestamps']     # (N,)
    gripper = root.get('gripper', None)
    imgs    = root.get('images', None)  # (N,3,H,W,3)

    num_frames = cmds.shape[0]
    print(f"Loaded {num_frames} frames from {args.zarr_path}")

    # Initialize image display windows
    window_names = ['left_wrist', 'right_wrist', 'front']
    if show_images and imgs is not None:
        for win in window_names:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    left_ctrl = right_ctrl = None
    executor = None

    # Initialize controllers and thread pool based on mode
    if mode == 'real':
        if not args.config:
            parser.error("--config parameter is required in real mode")
        config    = yaml.safe_load(open(args.config, 'r'))
        left_cfg  = config['controllers']['left']
        right_cfg = config['controllers']['right']

        # Parallel initialize left/right controllers
        with ThreadPoolExecutor(max_workers=2) as init_pool:
            init_futs = {
                init_pool.submit(SingleUniBiDexController, left_cfg, 'left'): 'left',
                init_pool.submit(SingleUniBiDexController, right_cfg, 'right'): 'right',
            }
            for fut in as_completed(init_futs):
                side = init_futs[fut]
                try:
                    ctrl = fut.result()
                    if side == 'left':
                        left_ctrl = ctrl
                    else:
                        right_ctrl = ctrl
                except Exception as e:
                    logger.error(f"{side} controller initialization failed: {e}")
                    sys.exit(1)
        executor = ThreadPoolExecutor(max_workers=2)
        print("Real mode: controllers parallel initialized.")
    else:
        print("Print-only mode: Not performing control, printing playback info only.")

    prev_t = None
    try:
        for idx in range(num_frames):
            # Play according to timestamps
            cur_t = float(ts[idx])
            if prev_t is not None:
                dt = cur_t - prev_t
                if dt > 0:
                    time.sleep(dt)
            prev_t = cur_t

            left_cmd, right_cmd = cmds[idx]
            left_open, right_open = gripper[idx]

            # In real mode, call controllers in parallel
            if mode == 'real':
                control_futs = {
                    executor.submit(left_ctrl.step_with_external_state,  left_cmd.tolist(),  left_open, teleop=False): 'L',
                    executor.submit(right_ctrl.step_with_external_state, right_cmd.tolist(), right_open, teleop=False): 'R',
                }
                for fut in as_completed(control_futs):
                    side = control_futs[fut]
                    try:
                        fut.result()
                    except Exception as e:
                        logger.error(f"{side} control exception at frame {idx}: {e}")


            # Log output (printed in both modes)
            print(f"\nFrame {idx+1}/{num_frames} @ {cur_t:.3f}s")
            print("  Left joints:  ", np.array2string(left_cmd, precision=3, separator=', '))
            print("  Right joints: ", np.array2string(right_cmd, precision=3, separator=', '))
            print(f"  Left gripper opening:  {left_open:.3f}")
            print(f"  Right gripper opening: {right_open:.3f}")

            # Display teleoperation images
            if show_images and imgs is not None:
                frame_imgs = np.array(imgs[idx])  # (3,H,W,3)
                for i, win in enumerate(window_names):
                    cv2.imshow(win, frame_imgs[i])
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    print("Playback aborted by user via 'q'.")
                    break

    except KeyboardInterrupt:
        print("Playback aborted by user.")
    finally:
        if mode == 'real':
            executor.shutdown(wait=True)
            # left_ctrl.shutdown()
            # right_ctrl.shutdown()
            print("Controllers shutdown.")
        if show_images and imgs is not None:
            cv2.destroyAllWindows()
        print("Playback finished.")

if __name__ == "__main__":
    main()
