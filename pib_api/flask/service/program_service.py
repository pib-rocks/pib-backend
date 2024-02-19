import os
from model.program_model import Program

PYTHON_CODE_PATH_TEMPL = "/home/pib/cerebra_programs/%s.py"

def create_empty_python_code_file(program_number):
    open(PYTHON_CODE_PATH_TEMPL % program_number, "w").close()

def write_to_python_code_file(program_number, code):
    file = open(PYTHON_CODE_PATH_TEMPL % program_number, "w")
    file.write(code)
    file.close()

def delete_python_code_file(program_number):
    os.remove(PYTHON_CODE_PATH_TEMPL % program_number)