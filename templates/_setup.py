def _entry_point(exec_name:str) -> str:
    return f"{exec_name} = {package_name}.{exec_name}:main"

def _data_file(to_:list[str], from_:str) -> tuple[str, list[str]]:
    return (to_, from_)