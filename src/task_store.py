"""
Task history database for storing and retrieving task executions.

This module provides storage for task execution data using SQLite for structured
data and Chroma for vector embeddings, enabling scene-agnostic similarity search.
"""

import os
import json
import sqlite3
from pathlib import Path
from typing import Dict, Any, List, Optional
from dataclasses import dataclass

from . import config
from .scene_loader import SceneConfig


# Color name mapping for RGBA values
COLOR_MAP = {
    (1, 0, 0): "red",
    (0, 1, 0): "green",
    (0, 0, 1): "blue",
    (1, 1, 0): "yellow",
    (1, 0, 1): "magenta",
    (0, 1, 1): "cyan",
    (1, 1, 1): "white",
    (0, 0, 0): "black",
    (1, 0.5, 0): "orange",
    (0.5, 0, 0.5): "purple",
}


def rgba_to_color_name(rgba: List[float]) -> str:
    """Convert RGBA values to a color name."""
    # Round to nearest 0.5 for matching
    r, g, b = round(rgba[0] * 2) / 2, round(rgba[1] * 2) / 2, round(rgba[2] * 2) / 2

    # Try exact match first
    key = (rgba[0], rgba[1], rgba[2])
    if key in COLOR_MAP:
        return COLOR_MAP[key]

    # Try rounded match
    key = (r, g, b)
    if key in COLOR_MAP:
        return COLOR_MAP[key]

    # Default to describing the color
    if rgba[0] > 0.5 and rgba[1] < 0.5 and rgba[2] < 0.5:
        return "red"
    elif rgba[0] < 0.5 and rgba[1] > 0.5 and rgba[2] < 0.5:
        return "green"
    elif rgba[0] < 0.5 and rgba[1] < 0.5 and rgba[2] > 0.5:
        return "blue"
    elif rgba[0] > 0.5 and rgba[1] > 0.5 and rgba[2] < 0.5:
        return "yellow"
    else:
        return f"rgb({rgba[0]:.1f},{rgba[1]:.1f},{rgba[2]:.1f})"


@dataclass
class TaskExecution:
    """Represents a stored task execution."""
    id: int
    task_description: str
    plan_reasoning: Optional[str]
    plan_commands: List[Dict[str, Any]]
    execution_status: str
    steps_completed: int
    task_satisfied: Optional[bool]
    verification_reasoning: Optional[str]
    actual_state: Optional[str]
    expected_state: Optional[str]
    discrepancies: Optional[List[str]]
    object_count: int
    object_types: str
    object_colors: str
    objects: List[Dict[str, Any]]


class TaskStore:
    """Database for storing and retrieving task executions."""

    def __init__(self, db_folder: Optional[str] = None):
        """
        Initialize the task store.

        Args:
            db_folder: Folder for database files. Defaults to config.DATABASE_FOLDER.
        """
        self.db_folder = Path(db_folder or config.DATABASE_FOLDER)
        self.db_folder.mkdir(parents=True, exist_ok=True)

        self.db_path = self.db_folder / config.SQLITE_DB_FILE
        self.chroma_path = self.db_folder / config.CHROMA_PERSIST_DIR

        self._chroma_client = None
        self._collection = None

        # Initialize SQLite database
        self._init_db()

    def _init_db(self):
        """Initialize SQLite database with schema."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        # Create task_executions table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS task_executions (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp DATETIME DEFAULT CURRENT_TIMESTAMP,
                task_description TEXT NOT NULL,
                plan_reasoning TEXT,
                plan_commands JSON,
                execution_status TEXT CHECK(execution_status IN ('success', 'failed')),
                steps_completed INTEGER,
                task_satisfied BOOLEAN,
                verification_reasoning TEXT,
                actual_state TEXT,
                expected_state TEXT,
                discrepancies JSON,
                object_count INTEGER,
                object_types TEXT,
                object_colors TEXT
            )
        """)

        # Create object_states table
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS object_states (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                execution_id INTEGER NOT NULL,
                object_index INTEGER NOT NULL,
                object_type TEXT NOT NULL,
                rel_position_x REAL,
                rel_position_y REAL,
                rel_position_z REAL,
                color_name TEXT,
                color_r REAL,
                color_g REAL,
                color_b REAL,
                color_a REAL,
                scale REAL DEFAULT 1.0,
                FOREIGN KEY (execution_id) REFERENCES task_executions(id) ON DELETE CASCADE
            )
        """)

        # Create indexes
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_task_satisfied ON task_executions(task_satisfied)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_object_count ON task_executions(object_count)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_object_types ON task_executions(object_types)")
        cursor.execute("CREATE INDEX IF NOT EXISTS idx_object_states_execution ON object_states(execution_id)")

        conn.commit()
        conn.close()

    def _get_chroma_collection(self):
        """Get or create Chroma collection (lazy loading).

        Uses Chroma's default embedding function which doesn't require
        sentence-transformers or accelerate.
        """
        if self._collection is None:
            import chromadb
            from chromadb.utils import embedding_functions

            self._chroma_client = chromadb.PersistentClient(path=str(self.chroma_path))

            # Use Chroma's default embedding function (doesn't need sentence-transformers)
            default_ef = embedding_functions.DefaultEmbeddingFunction()

            self._collection = self._chroma_client.get_or_create_collection(
                name="task_embeddings",
                embedding_function=default_ef,
                metadata={"hnsw:space": "cosine"}
            )
        return self._collection

    def _compute_centroid(self, objects: List[Dict[str, Any]]) -> List[float]:
        """Compute centroid of object positions."""
        if not objects:
            return [0.0, 0.0, 0.0]

        x_sum = sum(obj['position'][0] for obj in objects)
        y_sum = sum(obj['position'][1] for obj in objects)
        z_sum = sum(obj['position'][2] for obj in objects)
        n = len(objects)

        return [x_sum / n, y_sum / n, z_sum / n]

    def record_execution(
        self,
        scene_config: SceneConfig,
        task_description: str,
        plan: Dict[str, Any],
        execution_result: Dict[str, Any],
        verification_result: Optional[Dict[str, Any]] = None
    ) -> int:
        """
        Record a task execution to the database.

        Args:
            scene_config: Scene configuration with objects
            task_description: Task description text
            plan: Validated plan with reasoning and commands
            execution_result: Result of plan execution
            verification_result: Optional verification result

        Returns:
            ID of the recorded execution
        """
        # Extract object information
        objects_info = []
        for obj in scene_config.objects:
            objects_info.append({
                'name': obj.name,
                'type': obj.type,
                'position': obj.position,
                'color': obj.color,
                'scale': obj.scale
            })

        # Compute centroid for relative positions
        centroid = self._compute_centroid(objects_info)

        # Extract object summaries
        object_types = ",".join(sorted(obj['type'] for obj in objects_info))
        object_colors = ",".join(sorted(rgba_to_color_name(obj['color']) for obj in objects_info))
        object_count = len(objects_info)

        # Extract plan data
        plan_reasoning = plan.get('reasoning', '')
        plan_commands = plan.get('commands', [])

        # Extract execution data
        execution_status = execution_result.get('status', 'failed')
        steps_completed = execution_result.get('steps_completed', 0)

        # Extract verification data
        task_satisfied = None
        verification_reasoning = None
        actual_state = None
        expected_state = None
        discrepancies = None

        if verification_result:
            task_satisfied = verification_result.get('task_satisfied')
            verification_reasoning = verification_result.get('reasoning')
            actual_state = verification_result.get('actual_state')
            expected_state = verification_result.get('expected_state')
            discrepancies = verification_result.get('discrepancies')

        # Insert into SQLite
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()

        cursor.execute("""
            INSERT INTO task_executions (
                task_description, plan_reasoning, plan_commands,
                execution_status, steps_completed,
                task_satisfied, verification_reasoning, actual_state, expected_state, discrepancies,
                object_count, object_types, object_colors
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            task_description,
            plan_reasoning,
            json.dumps(plan_commands),
            execution_status,
            steps_completed,
            task_satisfied,
            verification_reasoning,
            actual_state,
            expected_state,
            json.dumps(discrepancies) if discrepancies else None,
            object_count,
            object_types,
            object_colors
        ))

        execution_id = cursor.lastrowid

        # Insert object states with relative positions
        for idx, obj in enumerate(objects_info):
            color_name = rgba_to_color_name(obj['color'])
            rel_x = obj['position'][0] - centroid[0]
            rel_y = obj['position'][1] - centroid[1]
            rel_z = obj['position'][2] - centroid[2]

            cursor.execute("""
                INSERT INTO object_states (
                    execution_id, object_index, object_type,
                    rel_position_x, rel_position_y, rel_position_z,
                    color_name, color_r, color_g, color_b, color_a, scale
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                execution_id, idx, obj['type'],
                rel_x, rel_y, rel_z,
                color_name,
                obj['color'][0], obj['color'][1], obj['color'][2], obj['color'][3],
                obj['scale']
            ))

        conn.commit()
        conn.close()

        # Add to Chroma for similarity search
        self._add_to_chroma(
            execution_id=execution_id,
            task_description=task_description,
            task_satisfied=task_satisfied,
            object_types=object_types,
            object_colors=object_colors,
            object_count=object_count
        )

        return execution_id

    def _add_to_chroma(
        self,
        execution_id: int,
        task_description: str,
        task_satisfied: Optional[bool],
        object_types: str,
        object_colors: str,
        object_count: int
    ):
        """Add task to Chroma for similarity search."""
        collection = self._get_chroma_collection()

        # Add to collection - Chroma will generate embeddings automatically
        collection.add(
            ids=[f"exec_{execution_id}"],
            documents=[task_description],
            metadatas=[{
                "execution_id": execution_id,
                "task_satisfied": task_satisfied if task_satisfied is not None else False,
                "object_types": object_types,
                "object_colors": object_colors,
                "object_count": object_count
            }]
        )

    def find_similar_tasks(
        self,
        query: str,
        current_objects: Optional[List[Dict[str, Any]]] = None,
        n_results: int = None,
        success_only: bool = True
    ) -> List[TaskExecution]:
        """
        Find similar past tasks using semantic search.

        Args:
            query: Query text (user message or task description)
            current_objects: Optional list of current objects for filtering
            n_results: Number of results to return. Defaults to config.RAG_NUM_EXAMPLES.
            success_only: Only return successful executions

        Returns:
            List of similar TaskExecution objects
        """
        n_results = n_results or config.RAG_NUM_EXAMPLES
        collection = self._get_chroma_collection()

        # Build where filter
        where_filter = None
        if success_only:
            where_filter = {"task_satisfied": True}

        # Query Chroma - it will generate embeddings automatically
        try:
            results = collection.query(
                query_texts=[query],
                n_results=n_results * 2,  # Over-fetch for filtering
                where=where_filter
            )
        except Exception:
            # Collection might be empty
            return []

        if not results or not results['ids'] or not results['ids'][0]:
            return []

        # Extract execution IDs and filter by similarity threshold
        execution_ids = []
        for idx, (doc_id, distance) in enumerate(zip(results['ids'][0], results['distances'][0])):
            # Chroma returns distance (lower = more similar for cosine)
            if distance <= config.RAG_SIMILARITY_THRESHOLD:
                exec_id = int(doc_id.replace("exec_", ""))
                execution_ids.append(exec_id)

        # Fetch full execution data from SQLite
        executions = []
        for exec_id in execution_ids[:n_results]:
            execution = self._fetch_execution(exec_id)
            if execution:
                executions.append(execution)

        return executions

    def _fetch_execution(self, execution_id: int) -> Optional[TaskExecution]:
        """Fetch a task execution from SQLite by ID."""
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        cursor = conn.cursor()

        # Fetch execution
        cursor.execute("""
            SELECT * FROM task_executions WHERE id = ?
        """, (execution_id,))

        row = cursor.fetchone()
        if not row:
            conn.close()
            return None

        # Fetch objects
        cursor.execute("""
            SELECT * FROM object_states WHERE execution_id = ? ORDER BY object_index
        """, (execution_id,))

        object_rows = cursor.fetchall()
        conn.close()

        # Parse JSON fields
        plan_commands = json.loads(row['plan_commands']) if row['plan_commands'] else []
        discrepancies = json.loads(row['discrepancies']) if row['discrepancies'] else None

        # Build objects list
        objects = []
        for obj_row in object_rows:
            objects.append({
                'index': obj_row['object_index'],
                'type': obj_row['object_type'],
                'rel_position': [obj_row['rel_position_x'], obj_row['rel_position_y'], obj_row['rel_position_z']],
                'color_name': obj_row['color_name'],
                'color': [obj_row['color_r'], obj_row['color_g'], obj_row['color_b'], obj_row['color_a']],
                'scale': obj_row['scale']
            })

        return TaskExecution(
            id=row['id'],
            task_description=row['task_description'],
            plan_reasoning=row['plan_reasoning'],
            plan_commands=plan_commands,
            execution_status=row['execution_status'],
            steps_completed=row['steps_completed'],
            task_satisfied=bool(row['task_satisfied']) if row['task_satisfied'] is not None else None,
            verification_reasoning=row['verification_reasoning'],
            actual_state=row['actual_state'],
            expected_state=row['expected_state'],
            discrepancies=discrepancies,
            object_count=row['object_count'],
            object_types=row['object_types'],
            object_colors=row['object_colors'],
            objects=objects
        )

    def get_execution_count(self) -> int:
        """Get total number of stored executions."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM task_executions")
        count = cursor.fetchone()[0]
        conn.close()
        return count

    def get_successful_execution_count(self) -> int:
        """Get number of successful executions."""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute("SELECT COUNT(*) FROM task_executions WHERE task_satisfied = 1")
        count = cursor.fetchone()[0]
        conn.close()
        return count
