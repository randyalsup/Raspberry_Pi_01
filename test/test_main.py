import os
import sys

# ensure src is importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))

import main


def test_round_up_1dp():
    assert main.round_up_1dp(1.01) == 1.1
    assert main.round_up_1dp(2.0) == 2.0
    assert main.round_up_1dp(2.01) == 2.1
