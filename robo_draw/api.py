import tempfile
import threading
from contextlib import asynccontextmanager
from enum import Enum
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import BackgroundTasks, FastAPI, File, Form, HTTPException, UploadFile
from mephew_python_commons import LoggerFactory
from pydantic import BaseModel

from .core import DrawOptions, RoboDrawer, RoboDrawerSettings


class RobotStatus(str, Enum):
    IDLE = "idle"
    CONNECTING = "connecting"
    DRAWING = "drawing"
    ERROR = "error"
    NOT_CONNECTED = "not_connected"


class DrawRequest(BaseModel):
    frame_item_name: str = "Drawing Frame"
    tool_item_name: str = "Drawing Tool"
    use_visual_simulation: bool = False
    pixel_item_name: Optional[str] = "Pixel"
    board_item_name: Optional[str] = "Drawing Board"
    approach_dist: int = -10
    scale: float = 1.0
    resolution: float = 1.0


class StatusResponse(BaseModel):
    status: RobotStatus
    message: str
    current_file: Optional[str] = None


class DrawResponse(BaseModel):
    success: bool
    message: str


# Global state
_drawer: Optional[RoboDrawer] = None
_status: RobotStatus = RobotStatus.NOT_CONNECTED
_status_message: str = "Robot not connected"
_current_file: Optional[str] = None
_lock = threading.Lock()
_settings: Optional[RoboDrawerSettings] = None
_logger = None


def get_logger():
    global _logger
    if _logger is None:
        _logger = LoggerFactory(log_files_prefix="RoboDraw").get_logger("API")
    return _logger


@asynccontextmanager
async def lifespan(app: FastAPI):
    global _settings
    _settings = RoboDrawerSettings()
    get_logger().info("RoboDraw API started")
    yield
    get_logger().info("RoboDraw API shutting down")


app = FastAPI(
    title="RoboDraw API",
    description="API do zdalnego wysyłania plików SVG i sterowania robotem rysującym",
    version="1.0.0",
    lifespan=lifespan,
)


@app.get("/status", response_model=StatusResponse)
async def get_status():
    """Pobierz aktualny status robota."""
    return StatusResponse(
        status=_status,
        message=_status_message,
        current_file=_current_file,
    )


@app.post("/connect")
async def connect_robot(
    robot_name: str = Form(default="Hanwha HCR-3A"),
    run_on_robot: bool = Form(default=False),
    force_robot: bool = Form(default=False),
):
    """Połącz się z robotem."""
    global _drawer, _status, _status_message

    with _lock:
        if _status == RobotStatus.DRAWING:
            raise HTTPException(status_code=409, detail="Robot is currently drawing")

        try:
            _status = RobotStatus.CONNECTING
            _status_message = f"Connecting to robot '{robot_name}'..."

            _drawer = RoboDrawer(_settings)
            _drawer.connect_robot(run_on_robot, robot_name, force_robot)

            _status = RobotStatus.IDLE
            _status_message = f"Connected to robot '{robot_name}'"

            return {"success": True, "message": _status_message}

        except Exception as e:
            _status = RobotStatus.ERROR
            _status_message = f"Connection failed: {str(e)}"
            _drawer = None
            raise HTTPException(status_code=500, detail=_status_message)


def _draw_task(svg_path: Path, draw_options: DrawOptions):
    """Background task to execute drawing."""
    global _status, _status_message, _current_file

    try:
        _status = RobotStatus.DRAWING
        _status_message = f"Drawing {svg_path.name}..."
        _current_file = svg_path.name

        get_logger().info(f"Starting draw task for {svg_path}")
        _drawer.draw(draw_options)

        _status = RobotStatus.IDLE
        _status_message = f"Successfully drew {svg_path.name}"
        get_logger().info(f"Draw task completed for {svg_path}")

    except Exception as e:
        _status = RobotStatus.ERROR
        _status_message = f"Drawing failed: {str(e)}"
        get_logger().error(f"Draw task failed: {e}")

    finally:
        _current_file = None
        # Clean up temp file
        try:
            if svg_path.exists():
                svg_path.unlink()
        except Exception:
            pass


@app.post("/draw", response_model=DrawResponse)
async def draw_svg(
    background_tasks: BackgroundTasks,
    svg_file: UploadFile = File(..., description="Plik SVG do narysowania"),
    frame_item_name: str = Form(default="Drawing Frame"),
    tool_item_name: str = Form(default="Drawing Tool"),
    use_visual_simulation: bool = Form(default=False),
    pixel_item_name: str = Form(default="Pixel"),
    board_item_name: str = Form(default="Drawing Board"),
    approach_dist: int = Form(default=-10),
    scale: float = Form(default=1.0),
    resolution: float = Form(default=1.0),
):
    """
    Prześlij plik SVG i rozpocznij rysowanie.
    
    Rysowanie odbywa się w tle, można sprawdzić status przez GET /status.
    """
    global _status

    if _drawer is None:
        raise HTTPException(
            status_code=400,
            detail="Robot not connected. Call POST /connect first.",
        )

    with _lock:
        if _status == RobotStatus.DRAWING:
            raise HTTPException(
                status_code=409,
                detail="Robot is already drawing. Wait for completion.",
            )

        if not svg_file.filename.endswith(".svg"):
            raise HTTPException(
                status_code=400,
                detail="File must be an SVG file.",
            )

        # Save uploaded file to temp location
        try:
            temp_dir = Path(tempfile.gettempdir()) / "robo_draw"
            temp_dir.mkdir(exist_ok=True)
            temp_path = temp_dir / svg_file.filename

            content = await svg_file.read()
            temp_path.write_bytes(content)

            get_logger().info(f"Saved uploaded SVG to {temp_path}")

        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to save uploaded file: {str(e)}",
            )

        draw_options = DrawOptions(
            svg_path=temp_path,
            frame_item_name=frame_item_name,
            tool_item_name=tool_item_name,
            use_visual_simulation=use_visual_simulation,
            pixel_item_name=pixel_item_name if use_visual_simulation else None,
            board_item_name=board_item_name if use_visual_simulation else None,
            approach_dist=approach_dist,
            scale=scale,
            resolution=resolution,
        )

        # Start drawing in background
        _status = RobotStatus.DRAWING
        background_tasks.add_task(_draw_task, temp_path, draw_options)

        return DrawResponse(
            success=True,
            message=f"Drawing started for {svg_file.filename}. Check /status for progress.",
        )


@app.post("/draw/sync", response_model=DrawResponse)
async def draw_svg_sync(
    svg_file: UploadFile = File(..., description="Plik SVG do narysowania"),
    frame_item_name: str = Form(default="Drawing Frame"),
    tool_item_name: str = Form(default="Drawing Tool"),
    use_visual_simulation: bool = Form(default=False),
    pixel_item_name: str = Form(default="Pixel"),
    board_item_name: str = Form(default="Drawing Board"),
    approach_dist: int = Form(default=-10),
    scale: float = Form(default=1.0),
    resolution: float = Form(default=1.0),
):
    """
    Prześlij plik SVG i rysuj synchronicznie (czekaj na zakończenie).
    
    UWAGA: To żądanie może trwać długo w zależności od złożoności SVG.
    """
    global _status, _status_message, _current_file

    if _drawer is None:
        raise HTTPException(
            status_code=400,
            detail="Robot not connected. Call POST /connect first.",
        )

    with _lock:
        if _status == RobotStatus.DRAWING:
            raise HTTPException(
                status_code=409,
                detail="Robot is already drawing. Wait for completion.",
            )

        if not svg_file.filename.endswith(".svg"):
            raise HTTPException(
                status_code=400,
                detail="File must be an SVG file.",
            )

        # Save uploaded file to temp location
        try:
            temp_dir = Path(tempfile.gettempdir()) / "robo_draw"
            temp_dir.mkdir(exist_ok=True)
            temp_path = temp_dir / svg_file.filename

            content = await svg_file.read()
            temp_path.write_bytes(content)

            get_logger().info(f"Saved uploaded SVG to {temp_path}")

        except Exception as e:
            raise HTTPException(
                status_code=500,
                detail=f"Failed to save uploaded file: {str(e)}",
            )

        draw_options = DrawOptions(
            svg_path=temp_path,
            frame_item_name=frame_item_name,
            tool_item_name=tool_item_name,
            use_visual_simulation=use_visual_simulation,
            pixel_item_name=pixel_item_name if use_visual_simulation else None,
            board_item_name=board_item_name if use_visual_simulation else None,
            approach_dist=approach_dist,
            scale=scale,
            resolution=resolution,
        )

        try:
            _status = RobotStatus.DRAWING
            _status_message = f"Drawing {svg_file.filename}..."
            _current_file = svg_file.filename

            _drawer.draw(draw_options)

            _status = RobotStatus.IDLE
            _status_message = f"Successfully drew {svg_file.filename}"
            _current_file = None

            return DrawResponse(
                success=True,
                message=f"Successfully completed drawing {svg_file.filename}",
            )

        except Exception as e:
            _status = RobotStatus.ERROR
            _status_message = f"Drawing failed: {str(e)}"
            _current_file = None
            raise HTTPException(
                status_code=500,
                detail=f"Drawing failed: {str(e)}",
            )

        finally:
            # Clean up temp file
            try:
                if temp_path.exists():
                    temp_path.unlink()
            except Exception:
                pass


@app.post("/stop")
async def stop_robot():
    """Zatrzymaj robota (jeśli możliwe)."""
    # Note: RoboDK doesn't have a simple "stop" command that works mid-movement
    # This is a placeholder for potential future implementation
    if _drawer is None:
        raise HTTPException(status_code=400, detail="Robot not connected")

    return {"success": False, "message": "Stop functionality not yet implemented"}


@app.post("/home")
async def go_home():
    """Wyślij robota do pozycji domowej."""
    global _status, _status_message

    if _drawer is None:
        raise HTTPException(status_code=400, detail="Robot not connected")

    with _lock:
        if _status == RobotStatus.DRAWING:
            raise HTTPException(status_code=409, detail="Robot is currently drawing")

        try:
            _status = RobotStatus.DRAWING
            _status_message = "Moving to home position..."

            _drawer._go_home()

            _status = RobotStatus.IDLE
            _status_message = "Robot at home position"

            return {"success": True, "message": "Robot moved to home position"}

        except Exception as e:
            _status = RobotStatus.ERROR
            _status_message = f"Failed to move home: {str(e)}"
            raise HTTPException(status_code=500, detail=_status_message)


def run_server(host: str = "0.0.0.0", port: int = 8000):
    """Uruchom serwer API."""
    uvicorn.run(app, host=host, port=port)


if __name__ == "__main__":
    run_server()
