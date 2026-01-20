"""
File utilities for the Universal Robot Control project.
Provides intelligent file and directory searching with pathlib.

Created by Sandor Burian with the help of GitHub Copilot (Claude Sonnet 4)
"""

from pathlib import Path
from typing import Union, List


def find_config_directory(config_dir: str, package_name: str = None) -> Path:
    """
    Find config directory with intelligent search.
    
    Args:
        config_dir: Config directory name or path
        package_name: Optional package name to search in (e.g., 'SO100_Robot')
    
    Returns:
        Path object to the config directory
    """
    config_path = Path(config_dir)
    
    if config_path.is_absolute():
        return config_path
        
    # If package_name provided, search in that package's directory
    if package_name:
        current_dir = Path(__file__).parent.parent  # Go up to package root
        package_dir = current_dir / "drivers" / package_name
        package_config_path = package_dir / config_dir
        
        if package_config_path.exists():
            return package_config_path
    
    # Fallback: relative to current working directory
    cwd_config_path = Path.cwd() / config_dir
    if cwd_config_path.exists():
        return cwd_config_path
        
    print(f"Warning: Config directory not found. Searched: {config_path}")
    return config_path  # Return original even if doesn't exist


def find_file(base_path: Union[str, Path], filename: str, fallback_paths: List[str] = None) -> Path:
    """
    Find a file in base path or fallback locations.
    
    Args:
        base_path: Primary directory to search in
        filename: Name of file to find
        fallback_paths: List of fallback file paths to search
    
    Returns:
        Path object to the found file
    """
    base_path = Path(base_path)
    fallback_paths = fallback_paths or []
    
    # Primary location: in base path
    primary_path = base_path / filename
    if primary_path.exists():
        return primary_path
        
    # Fallback locations
    for fallback in fallback_paths:
        fallback_path = Path(fallback)
        if fallback_path.exists():
            return fallback_path
            
    # If not found anywhere, return the primary one (for error handling)
    print(f"Warning: {filename} not found. Searched in: {primary_path}, {fallback_paths}")
    return primary_path


def find_project_file(filename: str, search_dirs: List[str] = None) -> Path:
    """
    Find a file anywhere in the project with common search locations.
    
    Args:
        filename: Name of file to find
        search_dirs: Additional directories to search
    
    Returns:
        Path object to the found file
    """
    search_dirs = search_dirs or []
    
    # Common project locations
    common_locations = [
        f"demo/{filename}",
        f"package/drivers/SO100_Robot/config/{filename}",
        f"package/drivers/AR4/config/{filename}",
        f"config/{filename}",
        f"package/config/{filename}",
        filename  # Current directory
    ]
    
    # Add custom search directories
    all_locations = search_dirs + common_locations
    
    for location in all_locations:
        path = Path(location)
        if path.exists():
            return path
            
    print(f"Warning: {filename} not found in project. Searched: {all_locations}")
    return Path(filename)  # Return as-is for error handling


def ensure_directory(directory_path: Union[str, Path]) -> Path:
    """
    Ensure directory exists, create if it doesn't.
    
    Args:
        directory_path: Path to directory
        
    Returns:
        Path object to the directory
    """
    path = Path(directory_path)
    path.mkdir(parents=True, exist_ok=True)
    return path


def get_project_root() -> Path:
    """
    Get the project root directory.
    
    Returns:
        Path object to project root
    """
    current = Path(__file__).parent
    
    # Look for markers of project root
    while current != current.parent:
        # Check for common project root markers
        if any((current / marker).exists() for marker in [
            'setup.py', 'pyproject.toml', 'requirements.txt', '.git', 'README.md'
        ]):
            return current
        current = current.parent
    
    # Fallback to current working directory
    return Path.cwd()


def resolve_config_path(relative_path: str, package_name: str = None) -> Path:
    """
    Resolve a config file path intelligently.
    
    Args:
        relative_path: Relative path to config file
        package_name: Package name to search in
        
    Returns:
        Resolved path to config file
    """
    # Try package-specific config first
    if package_name:
        config_dir = find_config_directory("config", package_name)
        package_path = config_dir / relative_path
        if package_path.exists():
            return package_path
    
    # Try project-wide locations
    project_root = get_project_root()
    project_paths = [
        project_root / "config" / relative_path,
        project_root / relative_path,
        Path.cwd() / relative_path
    ]
    
    for path in project_paths:
        if path.exists():
            return path
    
    print(f"Warning: Config file {relative_path} not found")
    return Path(relative_path)