# -*- coding: utf-8 -*-

from abc import ABC, abstractmethod

class LLMBase(ABC):
    """
    Abstract base class for all large language model implementations.
    """
    def __init__(self, model_name, **kwargs):
        """
        Initializes the model client.
        :param model_name: The name or identifier of the model to use.
        """
        self.model_name = model_name
        self._initialize(**kwargs)

    @abstractmethod
    def _initialize(self, **kwargs):
        """
        Perform any model-specific setup, like connecting to a client.
        This is called by the constructor.
        """
        pass

    @abstractmethod
    def query(self, system_prompt, user_prompt):
        """
        Queries the large language model.

        :param system_prompt: The system prompt to guide the model's behavior.
        :param user_prompt: The user's input/question.
        :return: A tuple containing (success, response_text, error_message).
                 - success (bool): True if the query was successful, False otherwise.
                 - response_text (str): The model's generated text.
                 - error_message (str): An error message if the query failed.
        """
        pass