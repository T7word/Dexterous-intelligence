import os
import sys
sys.path.insert(0, os.path.abspath('../../'))

project = 'DexRobot Kinematics'
copyright = '2025, DexRobot Inc.'
author = 'DexRobot Inc.'

# The full version, including alpha/beta/rc tags
release = '0.1.0'

# Extensions
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.mathjax',
    'sphinx.ext.intersphinx',
]

# Theme
html_theme = 'sphinx_rtd_theme'
