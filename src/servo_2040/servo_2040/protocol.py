"""Protocol definitions for servo2040 communication."""
from enum import Enum
import json
from typing import Dict, Any, Optional

class CommandType(Enum):
    SERVO_POSITIONS = "servo_positions"
    STATUS_REQUEST = "status_request"
    ERROR = "error"
    STATUS = "status"
    TERMINATE = "terminate"

class Protocol:
    @staticmethod
    def encode_command(cmd_type: CommandType, payload: Dict[str, Any]) -> bytes:
        """Encode a command into bytes with a newline terminator."""
        command = {
            "type": cmd_type.value,
            "payload": payload
        }
        return (json.dumps(command) + "\n").encode('utf-8')
    
    @staticmethod
    def decode_command(data: bytes) -> Optional[tuple[CommandType, Dict[str, Any]]]:
        """Decode bytes into a command tuple (type, payload)."""
        try:
            decoded = json.loads(data.decode('utf-8').strip())
            cmd_type = CommandType(decoded["type"])
            return (cmd_type, decoded["payload"])
        except (json.JSONDecodeError, ValueError, KeyError, UnicodeDecodeError):
            return None

    @staticmethod
    def encode_servo_positions(positions: Dict[str, float]) -> bytes:
        """Helper to encode servo positions command."""
        return Protocol.encode_command(CommandType.SERVO_POSITIONS, {"positions": positions})
    
    @staticmethod
    def encode_status_request() -> bytes:
        """Helper to encode status request command."""
        return Protocol.encode_command(CommandType.STATUS_REQUEST, {})
    
    @staticmethod
    def encode_status(status: Dict[str, Any]) -> bytes:
        """Helper to encode status response."""
        return Protocol.encode_command(CommandType.STATUS, status)
    
    @staticmethod
    def encode_error(message: str) -> bytes:
        """Helper to encode error message."""
        return Protocol.encode_command(CommandType.ERROR, {"message": message})

    @staticmethod
    def encode_terminate() -> bytes:
        """Helper to encode termination command."""
        return Protocol.encode_command(CommandType.TERMINATE, {})
