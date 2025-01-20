#!/usr/bin/env python3

import openai
from transformers import GPT2LMHeadModel, GPT2Tokenizer

class CodeSuggestor:
    def __init__(self, model_name="EleutherAI/gpt-neo-1.3B"):
        # Load pre-trained model from Hugging Face or OpenAI
        self.model_name = model_name
        self.model = GPT2LMHeadModel.from_pretrained(model_name)
        self.tokenizer = GPT2Tokenizer.from_pretrained(model_name)
    
    def get_code_suggestion(self, code_context):
        """
        Given a code context, return the model's suggestion.
        """
        inputs = self.tokenizer.encode(code_context, return_tensors="pt")
        outputs = self.model.generate(inputs, max_length=500, num_return_sequences=1, temperature=0.7)

        suggestion = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
        return suggestion

    def get_openai_suggestion(self, prompt, api_key):
        """
        Use OpenAI API to get suggestions (if OpenAI model is preferred).
        """
        openai.api_key = api_key
        response = openai.Completion.create(
            engine="text-davinci-003",
            prompt=prompt,
            max_tokens=100,
            n=1,
            stop=None,
            temperature=0.5,
        )
        return response.choices[0].text.strip()

# Test LLM code suggestion
if __name__ == "__main__":
    code_context = "def move_to_position(position):\n    # Move robot arm to the position"
    suggestor = CodeSuggestor()
    suggestion = suggestor.get_code_suggestion(code_context)
    print("Suggested Code:\n", suggestion)