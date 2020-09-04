import os

# return abs path to src/pam_models/config
def get_config_folder():
    from catkin_pkg import workspaces
    packages = workspaces.get_spaces()
    package_path = [ p for p in packages
                     if p.endswith("pam_models") ][0]
    return os.path.join(package_path,
                        "config")
                   

def get_config_path(config):
    config_folder = get_config_folder()
    config_file = config_folder+os.sep+config+".json"
    if not os.path.isfile(config_file):
        raise FileNotFoundError("failed to find pam_models config file "+config_file)
    return config_file

def get_default_config_path():
    return get_config_path("hill")
