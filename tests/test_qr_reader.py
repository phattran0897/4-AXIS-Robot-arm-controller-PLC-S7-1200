import numpy as np
import pytest
import cv2

from src.ai.qr_reader import QRReader


class TestQRReader:
    def test_debounce_logic_success(self):
        reader = QRReader(debounce_frames=3)
        
        # We can mock detectAndDecode to return predictable values
        class MockDetector:
            def __init__(self):
                self.val = ""
            def detectAndDecode(self, frame):
                return self.val, np.array([[0,0], [1,0], [1,1], [0,1]]), None

        reader._detector = MockDetector()
        
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        
        # Frame 1 - Value A
        reader._detector.val = "A1"
        assert reader.decode_qr(frame) is None
        
        # Frame 2 - Value A
        assert reader.decode_qr(frame) is None
        
        # Frame 3 - Value A (Debounce reached)
        assert reader.decode_qr(frame) == "A1"

    def test_debounce_logic_reset_on_change(self):
        reader = QRReader(debounce_frames=3)
        
        class MockDetector:
            def __init__(self):
                self.val = ""
            def detectAndDecode(self, frame):
                return self.val, np.array([[0,0], [1,0], [1,1], [0,1]]), None

        reader._detector = MockDetector()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        
        # Frame 1 - Value A
        reader._detector.val = "A1"
        assert reader.decode_qr(frame) is None
        
        # Frame 2 - Value B
        reader._detector.val = "B2"
        assert reader.decode_qr(frame) is None
        
        # Frame 3 - Value B
        assert reader.decode_qr(frame) is None
        
        # Frame 4 - Value B (Debounce reached for B2)
        assert reader.decode_qr(frame) == "B2"

    def test_debounce_logic_reset_on_none(self):
        reader = QRReader(debounce_frames=3)
        
        class MockDetector:
            def __init__(self):
                self.val = ""
            def detectAndDecode(self, frame):
                return self.val, np.array([[0,0], [1,0], [1,1], [0,1]]), None

        reader._detector = MockDetector()
        frame = np.zeros((10, 10, 3), dtype=np.uint8)
        
        # Frame 1 - Value A
        reader._detector.val = "A1"
        assert reader.decode_qr(frame) is None
        
        # Frame 2 - Nothing
        reader._detector.val = ""
        assert reader.decode_qr(frame) is None
        
        # Frame 3 - Value A (Starts over)
        reader._detector.val = "A1"
        assert reader.decode_qr(frame) is None
