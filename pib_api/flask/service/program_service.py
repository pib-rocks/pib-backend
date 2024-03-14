import os
from app.app import app


def create_empty_python_code_file(program_number):
    open(_get_code_filepath(program_number), "w").close()

def write_to_python_code_file(program_number, code):
    with open(_get_code_filepath(program_number), "w", encoding="utf-8") as f:
        f.write(code)

def delete_python_code_file(program_number):
    os.remove(_get_code_filepath(program_number))

def _get_code_filepath(program_number):
    return os.path.join(app.config['PYTHON_CODE_DIR'], f"{program_number}.py")