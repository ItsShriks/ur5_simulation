
import os
from flask import Flask, request, jsonify
from flask_cors import CORS
import openai

# Initialize Flask app
app = Flask(__name__)
CORS(app)

# OpenAI API key
openai.api_key = "api_key"


# Directory containing Python code
CODE_DIRECTORY = "/catkin_ws/src/my_pkg"

# Function to read all Python files
def read_python_files(directory):
    python_files = {}
    for root, _, files in os.walk(directory):
        for file in files:
            if file.endswith(".py"):
                with open(os.path.join(root, file), "r") as f:
                    python_files[file] = f.read()
    return python_files

@app.route('/list_files', methods=['GET'])
def list_files():
    """Endpoint to list all Python files in the directory."""
    try:
        files = read_python_files(CODE_DIRECTORY)
        return jsonify({"success": True, "files": list(files.keys())})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

@app.route('/process_file', methods=['POST'])
def process_file():
    """Endpoint to process a specific file."""
    try:
        data = request.json
        file_name = data.get('file_name')
        task = data.get('task', 'Suggest improvements')

        # Read the content of the selected file
        files = read_python_files(CODE_DIRECTORY)
        code = files.get(file_name)
        if not code:
            return jsonify({"success": False, "error": "File not found"}), 404

        # Send code to OpenAI API
        prompt = f"{task} for the following Python code:\n\n{code}"
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": "You are an AI coding assistant."},
                {"role": "user", "content": prompt}
            ],
            max_tokens=500,
        )
        result = response.choices[0].message['content']
        return jsonify({"success": True, "result": result})
    except Exception as e:
        return jsonify({"success": False, "error": str(e)}), 500

if __name__ == '__main__':
    app.run(debug=True)
