#!/usr/bin/env python3

from transformers import pipeline

class CodeCoPilot:
    def __init__(self, model_name="distilgpt2"):  # Lighter model
        self.co_pilot = pipeline("text-generation", model=model_name)

    def get_code_suggestion(self, user_code_input):
        """
        Provide a code suggestion for the user's input.
        Args:
            user_code_input (str): The code the user is working on.
        Returns:
            str: Suggested code completion.
        """
        suggestion = self.co_pilot(user_code_input, max_length=100, num_return_sequences=1)
        return suggestion[0]["generated_text"]

# Example usage:
if __name__ == "__main__":
    co_pilot = CodeCoPilot()

    user_code_input = """
def move_robot_to_position(api, position):
    # Move the robot to the target position
    api.move_linear(target_position=position)
"""

    suggested_code = co_pilot.get_code_suggestion(user_code_input)
    print("Code Suggestion:\n", suggested_code)
