from setuptools import setup, find_packages

def main():
    setup(
        name="unibidex_client",
        version="0.1.0",
        packages=find_packages(),
        install_requires=[
            "numpy",
            "pynput",
            "h5py",
        ],
        long_description_content_type="text/plain",
    )

if __name__ == "__main__":
    main()