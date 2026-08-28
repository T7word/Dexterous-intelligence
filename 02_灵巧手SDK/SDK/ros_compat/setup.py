from setuptools import setup, find_packages

setup(
    name="ros-compat",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[],
    extras_require={
        'ros1': ['rospy'],
        'ros2': ['rclpy'],
        'docs': [
            'sphinx>=8.1.3',
            'sphinx-rtd-theme>=1.3.0',
            'sphinx-autodoc-typehints',
        ],
    },
    author="Dexrobot",
    author_email="lyw@dexrobot.com",
    description="A compatibility layer for ROS1 and ROS2 Python nodes",
    long_description=open('README.md').read(),
    long_description_content_type="text/markdown",
    url="https://github.com/dexrobot/ros-compat",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
)
