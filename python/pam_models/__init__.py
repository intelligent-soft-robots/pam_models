import pam_configuration
import os
import pathlib

_PACKAGE_NAME = "pam_models"


def get_config_folder() -> pathlib.Path:
    prefix = pathlib.Path(pam_configuration.get_path())
    return prefix / _PACKAGE_NAME


def get_config_path(config: str) -> pathlib.Path:
    config_folder = get_config_folder()
    config_file = config_folder / (str(config) + ".json")
    if not config_file.is_file():
        raise FileNotFoundError(
            "failed to find pam_models config file {}".format(config_file)
        )
    return config_file


def get_default_config_path() -> pathlib.Path:
    return get_config_path("hill")
