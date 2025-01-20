from flask import Flask, request, jsonify
from model import generate_code_suggestions

app = Flask(__name__)

@app.route('/get_suggestions', methods=['POST'])
def get_suggestions():
    data = request.json
    code = data.get('code', '')
    
    if not code:
        return jsonify({"error": "No code provided"}), 400
    
    # Get code suggestions from the LLM
    suggestions = generate_code_suggestions(code)
    
    return jsonify({"suggestions": suggestions})

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)