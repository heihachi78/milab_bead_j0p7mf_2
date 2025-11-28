"""
RAG retriever for finding and formatting similar past task executions.

This module provides retrieval-augmented generation functionality for
interactive/console modes, helping the model learn from past successes.
"""

import json
from pathlib import Path
from typing import Dict, Any, List, Optional

from . import config
from .task_store import TaskStore, TaskExecution


class RAGRetriever:
    """Retrieves and formats similar past tasks for RAG."""

    def __init__(self, task_store: Optional[TaskStore] = None, logger=None):
        """
        Initialize the RAG retriever.

        Args:
            task_store: TaskStore instance. Creates new one if not provided.
            logger: SimulationLogger instance for logging RAG queries/results.
        """
        self.task_store = task_store or TaskStore()
        self.logger = logger
        self._context_template = None

    def _load_context_template(self) -> str:
        """Load the RAG context template from file."""
        if self._context_template is None:
            template_file = config.RAG_CONTEXT_TEMPLATE_FILE
            filepath = Path(__file__).parent.parent / 'prompts' / template_file

            if filepath.exists():
                with open(filepath, 'r') as f:
                    self._context_template = f.read()
            else:
                # Default template
                self._context_template = self._get_default_template()

        return self._context_template

    def _get_default_template(self) -> str:
        """Return default RAG context template."""
        return """## Relevant Past Successful Tasks

The following are examples of similar tasks that were successfully completed in the past.
Use these as guidance for planning your approach:

{examples}

---

"""

    def get_relevant_examples(
        self,
        user_message: str,
        current_objects: Optional[List[str]] = None,
        n_examples: int = None
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant past task examples.

        Args:
            user_message: User's message/request
            current_objects: List of current object names (optional)
            n_examples: Number of examples to retrieve

        Returns:
            List of example dictionaries with task, plan, and object info
        """
        n_examples = n_examples or config.RAG_NUM_EXAMPLES

        # Convert object names to object info if provided
        objects_info = None
        if current_objects:
            objects_info = [{'name': name} for name in current_objects]

        # Find similar tasks
        similar_tasks = self.task_store.find_similar_tasks(
            query=user_message,
            current_objects=objects_info,
            n_results=n_examples,
            success_only=True
        )

        # Format examples
        examples = []
        for task in similar_tasks:
            examples.append({
                'task_description': task.task_description,
                'plan_reasoning': task.plan_reasoning,
                'plan_commands': task.plan_commands,
                'object_types': task.object_types,
                'object_colors': task.object_colors,
                'object_count': task.object_count,
                'objects': task.objects
            })

        return examples

    def format_rag_context(self, examples: List[Dict[str, Any]]) -> str:
        """
        Format RAG examples into context string for injection into prompts.

        Args:
            examples: List of example dictionaries from get_relevant_examples

        Returns:
            Formatted context string
        """
        if not examples:
            return ""

        template = self._load_context_template()

        # Format each example
        formatted_examples = []
        for i, example in enumerate(examples, 1):
            formatted = self._format_single_example(i, example)
            formatted_examples.append(formatted)

        examples_text = "\n\n".join(formatted_examples)

        return template.format(examples=examples_text)

    def _format_single_example(self, index: int, example: Dict[str, Any]) -> str:
        """Format a single example for display."""
        lines = [f"### Example {index}: {example['task_description']}"]

        # Object configuration
        lines.append(f"**Objects:** {example['object_count']} objects ({example['object_types']})")
        lines.append(f"**Colors:** {example['object_colors']}")

        # Plan reasoning
        if example.get('plan_reasoning'):
            lines.append(f"**Reasoning:** {example['plan_reasoning']}")

        # Successful plan
        if example.get('plan_commands'):
            lines.append("**Successful Plan:**")
            lines.append("```json")
            lines.append(json.dumps(example['plan_commands'], indent=2))
            lines.append("```")

        return "\n".join(lines)

    def get_rag_context_for_message(
        self,
        user_message: str,
        current_objects: Optional[List[str]] = None
    ) -> str:
        """
        Get formatted RAG context for a user message.

        This is the main entry point for RAG integration.

        Args:
            user_message: User's message/request
            current_objects: List of current object names (optional)

        Returns:
            Formatted RAG context string, or empty string if no relevant examples
        """
        if not config.ENABLE_RAG:
            return ""

        # Check if we have any stored executions
        if self.task_store.get_successful_execution_count() == 0:
            return ""

        # Log the query
        if self.logger:
            self.logger.log_rag_query(user_message, current_objects)

        examples = self.get_relevant_examples(
            user_message=user_message,
            current_objects=current_objects
        )

        formatted_context = self.format_rag_context(examples) if examples else ""

        # Log the results
        if self.logger:
            self.logger.log_rag_results(examples, formatted_context)

        return formatted_context

    def get_stats(self) -> Dict[str, int]:
        """Get statistics about stored task executions."""
        return {
            'total_executions': self.task_store.get_execution_count(),
            'successful_executions': self.task_store.get_successful_execution_count()
        }
