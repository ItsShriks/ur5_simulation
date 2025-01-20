document.getElementById('get-suggestions').addEventListener('click', async () => {
    const codeInput = document.getElementById('code-input').value;
    const response = await fetch('http://localhost:5000/get_suggestions', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ code: codeInput }),
    });

    const data = await response.json();
    
    if (data.suggestions) {
        document.getElementById('suggestions-output').textContent = data.suggestions;
    } else {
        document.getElementById('suggestions-output').textContent = 'No suggestions available.';
    }
});