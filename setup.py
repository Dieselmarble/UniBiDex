import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="unibidex",
    version="0.0.1",
    author="Zhongxuan Li",
    author_email="lizhongxuanchina@gmail.com",
    description="software for UniBiDex",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Dieselmarble/UniBiDex",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires=">=3.8",
    license="MIT",
    install_requires=[
        "numpy",
    ],
)
