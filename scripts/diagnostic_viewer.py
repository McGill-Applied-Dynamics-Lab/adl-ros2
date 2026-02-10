from rich.console import Console
from arm_client.utils.diagnostics_viewer import DiagnosticViewer

console = Console()
console.clear()

viewer = DiagnosticViewer()
viewer.display()
