from openai import OpenAI
from openai.resources.beta.threads.threads import Thread
from openai.resources.beta.threads.runs.runs import Run
from openai.resources.beta.threads.messages import Message
from openai.pagination import SyncCursorPage
import time
import ast
import astor
import tkinter as tk
from tkinter import ttk
from typing_extensions import List, Optional
from enum import Enum
from pycram.failures import ReasoningError
import re


class Assistant(Enum):
    """
    Enum class to define the assistant IDs for the OpenAI API
    """

    # Generates the code within the `perform()` function of a given ActionPerformable.
    PERFORM = "asst_LkoSuvswUNGCQgrjXVI5Z1Y5"

    # Generates a comprehensive manual for how to write PyCRAM code.
    DOCS = "asst_gsAB5Ksi9RDl6ggUDXqyGNpE"


prompts = {
    'milk': "Given milk_desig, fridge_desig and table_desig, fetch me the milk from the fridge and put it on the table.",
    'place': "Place the object held in your left hand on the table_desig.",
    'transport': "TransportActionPerformable"
}


class OpenAIClient:
    """
    A wrapper for the OpenAI API client to interact with the assistant.
    """
    def __init__(self, assistant=Assistant.PERFORM):
        self.client = OpenAI()
        self.assistant_id = assistant.value
        self.thread: Optional[Thread] = None
        self.run: Optional[Run] = None

    def __init_thread_runner(self, prompt):
        self.thread = self.client.beta.threads.create(
            messages=[
                {
                    "role": "user",
                    "content": prompt
                }
            ]
        )
        self.run = self.client.beta.threads.runs.create(
            thread_id=self.thread.id,
            assistant_id=self.assistant_id
        )

    def run_query(self, prompt):
        self.__init_thread_runner(prompt)
        while self.run.status != 'completed':
            self.run = self.client.beta.threads.runs.retrieve(
                thread_id=self.thread.id,
                run_id=self.run.id
            )
            print(self.run.status)
            time.sleep(5)

    def get_response(self) -> SyncCursorPage[Message]:
        # Extract all messages of the thread response
        return self.client.beta.threads.messages.list(self.thread.id)


class CodeGenerator(OpenAIClient):
    """
    A class to generate the perform() function code body for Actions using the OpenAI API assistant.
    """
    code_response: str

    def __init__(self):
        super().__init__(Assistant.PERFORM)
        self.code_response = ""

    def extract_code_from_response(self) -> str:
        """
        Extract the generated code from the assistant response.
        The code is expected to be within triple backticks (```python) in the response message.
        """
        response = self.get_response()
        # Extract the last response message content as raw String
        message_text = response.data[0].content[0].text.value
        pattern = f"{re.escape('```python')}(.*?){re.escape('```')}"
        code = ''.join(re.findall(pattern, message_text, re.DOTALL))
        return code

    def generate_code(self, prompt: str) -> List[str]:
        """
        Generate the body of a `perform()` function for the specified ActionPeformable using the prepared assistant.

        cgen = CodeGenerator()
        cgen.generate_code('TransportActionPerformable')
        """
        self.run_query(prompt=prompt)
        self.code_response = self.extract_code_from_response()
        tree = ast.parse(self.code_response)
        refined_code = extract_function_body(tree)
        return refined_code


def extract_function_body(node: ast.Module) -> List[str]:
    """
    Investigate the AST tree and extract the body of the first function

    node: ast.Module The tree to investigate.
    """
    body = []
    for node in ast.walk(node):
        if isinstance(node, ast.FunctionDef):
            # Extract the body of the function
            body = [astor.to_source(stmt) for stmt in node.body]
            break  # TODO: Handle multiple functions in the code
    return body


code_gen: Optional[CodeGenerator] = None


def generate_perform_for_action(action_name: str) -> List[str]:
    """
    Generate the body of a `perform()` function for the specified ActionPeformable using the prepared assistant.

    generate_perform_for_action('TransportActionPerformable')
    """
    global code_gen
    if code_gen is None:
        code_gen = CodeGenerator()
    generated_code = code_gen.generate_code(action_name)
    if ask_permission(generated_code):
        return generated_code
    else:
        raise ReasoningError("Generated code denied for perform() function for action: " + action_name)


# --- Message box for asking permission ---

class ScrollableMessageBox(tk.Toplevel):
    """
    A scrollable message box with a "Yes" and "No" button to ask for user permission to proceed.
    """

    def __init__(self, title, message):
        self.root = tk.Tk()
        self.root.withdraw()  # Hide the root window

        tk.Toplevel.__init__(self)
        self.title(title)
        self.geometry("800x800")  # Set the size of the window

        # Create a scrollable text area
        text_area = tk.Text(self, wrap='word')
        text_area.insert(tk.END, message)
        text_area.pack(side='left', fill='both', expand=True)

        # Create a scrollbar and attach it to the text area
        scrollbar = ttk.Scrollbar(self, command=text_area.yview)
        scrollbar.pack(side='right', fill='y')
        text_area['yscrollcommand'] = scrollbar.set

        # Create a "Yes" button
        yes_button = ttk.Button(self, text="Yes", command=self.yes)
        yes_button.pack()

        # Create a "No" button
        no_button = ttk.Button(self, text="No", command=self.no)
        no_button.pack()

        self.result = None

    def yes(self):
        self.result = True
        self.destroy()

    def no(self):
        self.result = False
        self.destroy()


def ask_permission(message):
    """
    Create a message box displaying the message and asking for user permission to proceed.

    returns: True if the user clicks "Yes", False if the user clicks "No"
    """
    box = ScrollableMessageBox("Permission", message)
    box.wait_window()  # Wait for the user to close the window
    return box.result
