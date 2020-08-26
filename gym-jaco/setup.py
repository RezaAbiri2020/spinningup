
import setuptools
from pathlib import Path

setuptools.setup(
    name='gym_jaco',
    author="Reza Abiri",
    author_email="reza.abiri2020@gmail.com",
    version='0.0.6',
    description="An OpenAI Gym Env for jaco 7DOF",
    long_description=Path("README.md").read_text(),
    long_description_content_type="text/markdown",
    url="https://github.com/RezaAbiri2020/JacoPyBullet1",
    packages=setuptools.find_packages(include="gym_jaco*"),
    install_requires=['gym', 'pybullet', 'numpy'],  # And any other dependencies foo needs
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        ],
    python_requires='>=3.6'
)
