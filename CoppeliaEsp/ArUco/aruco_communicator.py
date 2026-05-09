"""
ArUco Communicator Module

This module provides inter-process communication between live.py (ArUco detection)
and follow_ball.py (PID controller) using JSON file sharing.

Classes:
    ArucoDataPublisher: Writes ArUco detection data to JSON files
    ArucoDataSubscriber: Reads and validates ArUco detection data from JSON files

Author: Copilot
Date: 2024
"""

import json
import threading
import time
from pathlib import Path
from typing import Dict, Optional, Any
import os


class ArucoDataPublisher:
    """
    Publishes ArUco detection data to JSON files for inter-process communication.
    
    Thread-safe writer for ArUco marker position, angle, and detection status.
    """
    
    def __init__(self, data_dir: str = "."):
        """
        Initialize the publisher.
        
        Args:
            data_dir (str): Directory where JSON files will be written
        """
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.lock = threading.Lock()
    
    def publish(self, car_id: int, data: Dict[str, Any]) -> bool:
        """
        Publish ArUco detection data for a specific car.
        
        Args:
            car_id (int): Car identifier
            data (dict): Detection data containing:
                - x (float): X position in meters
                - y (float): Y position in meters
                - angle (float): Rotation angle in degrees [-180, 180]
                - detected (bool): Whether car was detected in this frame
                - distance_to_target (float): Distance to target in meters
                - timestamp (float): Unix timestamp
        
        Returns:
            bool: True if publish was successful
        """
        try:
            with self.lock:
                file_path = self.data_dir / f"aruco_data_car_{car_id}.json"
                
                # Ensure timestamp
                if 'timestamp' not in data:
                    data['timestamp'] = time.time()
                
                with open(file_path, 'w') as f:
                    json.dump(data, f)
                
                return True
        except Exception as e:
            print(f"[ArUco Publisher ERROR] Failed to publish car {car_id} data: {e}")
            return False


class ArucoDataSubscriber:
    """
    Subscribes to ArUco detection data from JSON files.
    
    Thread-safe reader with automatic timeout detection for stale data.
    """
    
    def __init__(self, data_dir: str = ".", timeout: float = 2.0):
        """
        Initialize the subscriber.
        
        Args:
            data_dir (str): Directory where JSON files are located
            timeout (float): Maximum age of data in seconds before considering it stale
        """
        self.data_dir = Path(data_dir)
        self.timeout = timeout
        self.lock = threading.Lock()
        self.last_read = {}
    
    def subscribe(self, car_id: int) -> Optional[Dict[str, Any]]:
        """
        Read ArUco detection data for a specific car.
        
        Args:
            car_id (int): Car identifier
        
        Returns:
            dict: Detection data if available and not stale, None otherwise
            
        Data structure:
        {
            'x': float,                    # X position in meters
            'y': float,                    # Y position in meters
            'angle': float,                # Rotation angle in degrees
            'detected': bool,              # Detection status
            'distance_to_target': float,   # Distance in meters
            'timestamp': float,            # Unix timestamp
            'stale': bool                  # True if data is older than timeout
        }
        """
        try:
            with self.lock:
                file_path = self.data_dir / f"aruco_data_car_{car_id}.json"
                
                if not file_path.exists():
                    return None
                
                with open(file_path, 'r') as f:
                    data = json.load(f)
                
                # Check if data is stale
                timestamp = data.get('timestamp', 0)
                age = time.time() - timestamp
                data['stale'] = age > self.timeout
                
                self.last_read[car_id] = data
                return data
        except Exception as e:
            print(f"[ArUco Subscriber ERROR] Failed to read car {car_id} data: {e}")
            return None
    
    def get_last(self, car_id: int) -> Optional[Dict[str, Any]]:
        """
        Get the last successfully read data without reading file again.
        
        Args:
            car_id (int): Car identifier
        
        Returns:
            dict: Last read data or None if never read
        """
        return self.last_read.get(car_id)


def create_default_aruco_data(detected: bool = False) -> Dict[str, Any]:
    """
    Create a default ArUco data structure.
    
    Args:
        detected (bool): Whether car was detected
    
    Returns:
        dict: Default ArUco data
    """
    return {
        'x': 0.0,
        'y': 0.0,
        'angle': 0.0,
        'detected': detected,
        'distance_to_target': 0.0,
        'timestamp': time.time(),
        'stale': False
    }
