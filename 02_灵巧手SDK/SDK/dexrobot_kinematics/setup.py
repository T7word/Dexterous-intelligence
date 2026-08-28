from setuptools import setup, find_packages

setup(
    name="dexrobot_kinematics",
    version="0.1.0",
    description="Kinematics library for DexHand",
    author="DexRobot Inc.",
    packages=find_packages(),

    # Dependencies
    install_requires=[
        "numpy>=1.20.0",
        "pin>=2.6.0",  # For robot kinematics
        "matplotlib>=3.4.0",  # For visualization
        "pyyaml>=5.4.0",     # For config files
        "pytest>=6.0.0",     # For testing
    ],

    # Optional dependencies for development
    extras_require={
        "dev": [
            "black>=22.0.0",       # Code formatting
            "flake8>=4.0.0",       # Code linting
            "pytest-cov>=3.0.0",   # Test coverage
            "pytest-xdist>=2.4.0", # Parallel testing
        ],
        "docs": [
            "sphinx>=8.1.3",
            "sphinx-rtd-theme>=1.3.0",
            "sphinx-autodoc-typehints",
        ],
    },

    # Package data
    package_data={
        "dexrobot_kinematics": [
            "config/*.yaml",
        ],
    },
    include_package_data=True,

    # CLI scripts if needed
    entry_points={
        "console_scripts": [
            # Add CLI scripts here if needed
            # "dexrobot-viz=dexrobot_kinematics.cli.visualize:main",
        ],
    },

    # Metadata
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering :: Robotics",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
    python_requires=">=3.8",
)
