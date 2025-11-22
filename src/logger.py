import logging
import os
from datetime import datetime
from typing import Optional


class SimulationLogger:
    """
    Multi-channel logging system for robot simulation.

    Provides separate log files for:
    - LLM interactions (requests/responses)
    - Robot actions (movements, pick/place operations)
    - Application events (state changes, errors)

    Console output is limited to high-level progress messages.
    """

    def __init__(self, log_dir: str = "logs", session_name: Optional[str] = None):
        """
        Initialize the logging system with separate file handlers.

        Args:
            log_dir: Directory to store log files
            session_name: Optional session identifier (defaults to timestamp)
        """
        self.log_dir = log_dir

        # Create logs directory if it doesn't exist
        os.makedirs(log_dir, exist_ok=True)

        # Generate session identifier
        if session_name is None:
            session_name = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_name = session_name

        # Initialize loggers
        self.llm_logger = self._create_logger(
            "llm",
            f"{log_dir}/llm_{session_name}.log",
            logging.DEBUG
        )

        self.robot_logger = self._create_logger(
            "robot",
            f"{log_dir}/robot_{session_name}.log",
            logging.DEBUG
        )

        self.app_logger = self._create_logger(
            "app",
            f"{log_dir}/app_{session_name}.log",
            logging.INFO
        )

        # Console logger for important messages only
        self.console_logger = self._create_console_logger()

        self.log_app_info(f"Logging session started: {session_name}")

    def _create_logger(self, name: str, filepath: str, level: int) -> logging.Logger:
        """
        Create a logger with file handler.

        Args:
            name: Logger name identifier
            filepath: Path to log file
            level: Logging level (e.g., logging.DEBUG, logging.INFO)

        Returns:
            Configured logging.Logger instance
        """
        logger = logging.getLogger(f"sim.{name}")
        logger.setLevel(level)
        logger.propagate = False

        # Remove existing handlers
        logger.handlers.clear()

        # File handler with detailed formatting
        file_handler = logging.FileHandler(filepath, mode='w', encoding='utf-8')
        file_handler.setLevel(level)

        formatter = logging.Formatter(
            '%(asctime)s | %(levelname)-8s | %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        return logger

    def _create_console_logger(self) -> logging.Logger:
        """
        Create console logger for important messages only.

        Returns:
            Configured logging.Logger instance for console output
        """
        logger = logging.getLogger("sim.console")
        logger.setLevel(logging.INFO)
        logger.propagate = False

        # Remove existing handlers
        logger.handlers.clear()

        # Console handler with simple formatting
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)

        formatter = logging.Formatter('%(message)s')
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

        return logger

    # ============== LLM Logging Methods ==============

    def log_llm_request(self, system_prompt: str, user_prompt: str, model: str = "unknown"):
        """Log an LLM request with system and user prompts."""
        self.llm_logger.info("=" * 80)
        self.llm_logger.info(f"LLM REQUEST | Model: {model}")
        self.llm_logger.info("-" * 80)
        self.llm_logger.info("SYSTEM PROMPT:")
        self.llm_logger.info(system_prompt)
        self.llm_logger.info("-" * 80)
        self.llm_logger.info("USER PROMPT:")
        self.llm_logger.info(user_prompt)
        self.llm_logger.info("=" * 80)

    def log_llm_response(self, response: str, model: str = "unknown"):
        """Log an LLM response."""
        self.llm_logger.info("=" * 80)
        self.llm_logger.info(f"LLM RESPONSE | Model: {model}")
        self.llm_logger.info("-" * 80)
        self.llm_logger.info(response)
        self.llm_logger.info("=" * 80)
        self.llm_logger.info("")  # Empty line for readability

    def log_llm_error(self, error: str):
        """Log an LLM error."""
        self.llm_logger.error(f"LLM ERROR: {error}")

    # ============== Robot Logging Methods ==============

    def log_robot_action(self, action: str, details: Optional[str] = None):
        """Log a robot action."""
        msg = f"ACTION: {action}"
        if details:
            msg += f" | {details}"
        self.robot_logger.info(msg)

    def log_robot_movement(self, target_pos: list, current_pos: list, distance: float):
        """Log robot movement details."""
        self.robot_logger.debug(
            f"MOVEMENT | Target: {self._format_pos(target_pos)} | "
            f"Current: {self._format_pos(current_pos)} | Distance: {distance:.6f}m"
        )

    def log_robot_pick(self, object_name: str, position: list):
        """Log pick operation."""
        self.robot_logger.info(
            f"PICK | Object: {object_name} | Position: {self._format_pos(position)}"
        )
        self.console_logger.info(f"Picking up {object_name}")

    def log_robot_place(self, position: list, on_object: Optional[str] = None):
        """Log place operation."""
        if on_object:
            self.robot_logger.info(
                f"PLACE | On: {on_object} | Position: {self._format_pos(position)}"
            )
            self.console_logger.info(f"Placing on {on_object}")
        else:
            self.robot_logger.info(
                f"PLACE | Position: {self._format_pos(position)}"
            )
            self.console_logger.info(f"Placing at position {self._format_pos(position)}")

    def log_robot_gripper(self, state: str, position: float):
        """Log gripper state change."""
        self.robot_logger.info(f"GRIPPER | State: {state} | Position: {position:.4f}")

    def log_robot_stabilize(self, steps: int):
        """Log stabilization operation."""
        self.robot_logger.info(f"STABILIZE | Steps: {steps}")
        self.console_logger.info("Stabilizing robot...")

    def log_robot_convergence(self, iterations: int, final_distance: float):
        """Log movement convergence."""
        self.robot_logger.debug(
            f"CONVERGED | Iterations: {iterations} | Final distance: {final_distance:.6f}m"
        )

    # ============== Application Logging Methods ==============

    def log_app_info(self, message: str):
        """Log general application information."""
        self.app_logger.info(message)

    def log_app_warning(self, message: str):
        """Log application warning."""
        self.app_logger.warning(message)
        self.console_logger.warning(f"WARNING: {message}")

    def log_app_error(self, message: str):
        """Log application error."""
        self.app_logger.error(message)
        self.console_logger.error(f"ERROR: {message}")

    def log_app_state_change(self, old_state: str, new_state: str):
        """Log application state change."""
        self.app_logger.info(f"STATE CHANGE | {old_state} -> {new_state}")

    def log_app_simulation_start(self, time: float):
        """Log simulation start."""
        self.app_logger.info("=" * 80)
        self.app_logger.info(f"SIMULATION START | t={time}")
        self.app_logger.info("=" * 80)
        self.console_logger.info("=" * 40)
        self.console_logger.info(f"Simulation started (t={time})")
        self.console_logger.info("=" * 40)

    def log_app_simulation_end(self, time: float):
        """Log simulation end."""
        self.app_logger.info("=" * 80)
        self.app_logger.info(f"SIMULATION END | t={time}")
        self.app_logger.info("=" * 80)
        self.console_logger.info("=" * 40)
        self.console_logger.info(f"Simulation ended (t={time})")
        self.console_logger.info("=" * 40)

    def log_app_plan_validated(self, validation_result: str):
        """Log plan validation result."""
        self.app_logger.info(f"PLAN VALIDATED: {validation_result}")

    def log_app_object_loaded(self, object_name: str, position: list):
        """Log object loading."""
        self.app_logger.info(
            f"OBJECT LOADED | Name: {object_name} | Position: {self._format_pos(position)}"
        )

    def log_app_camera_capture(self, capture_type: str, filename: str):
        """Log camera capture."""
        self.app_logger.info(f"CAMERA CAPTURE | Type: {capture_type} | File: {filename}")

    # ============== Console-only Methods ==============

    def console_info(self, message: str):
        """Print important information to console only."""
        self.console_logger.info(message)

    def console_progress(self, message: str):
        """Print progress update to console."""
        self.console_logger.info(f">> {message}")

    # ============== Helper Methods ==============

    def _format_pos(self, position: list) -> str:
        """
        Format position array for logging.

        Args:
            position: Position list [x, y, z]

        Returns:
            Formatted position string with 3 decimal places
        """
        return f"[{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]"

    def close(self):
        """Close all log handlers."""
        for logger in [self.llm_logger, self.robot_logger, self.app_logger, self.console_logger]:
            for handler in logger.handlers:
                handler.close()
                logger.removeHandler(handler)
