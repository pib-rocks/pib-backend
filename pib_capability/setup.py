from setuptools import setup

setup(
    name="pib_capability",
    version="0.1",
    description="Per-robot capability manifests for pib",
    packages=["pib_capability"],
    package_data={"pib_capability": ["manifests/*.json"]},
)
