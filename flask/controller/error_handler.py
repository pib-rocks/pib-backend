from flask import jsonify

def handle_not_found_error(error):
    return jsonify({'error':"Entity not found. Please check your path parameter."}), 404

def handle_internal_server_error(error):
    return jsonify({'error': "Internal Server Error, please try later again."}), 500

def handle_not_implemented_error(error):
    return jsonify({'error': "Not implemented."}), 501