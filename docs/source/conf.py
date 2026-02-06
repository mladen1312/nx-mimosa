# Configuration file for the Sphinx documentation builder.
# NX-MIMOSA v5.0.0 Documentation

import os
import sys
sys.path.insert(0, os.path.abspath('../..'))

# -- Project information
project = 'NX-MIMOSA'
copyright = '2026, Nexellum d.o.o.'
author = 'Dr. Mladen Mešter'
release = '5.0.0'
version = '5.0'

# -- General configuration
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',
    'sphinx.ext.intersphinx',
    'sphinx.ext.mathjax',
    'sphinx_autodoc_typehints',
]

templates_path = ['_templates']
exclude_patterns = []

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'undoc-members': False,
    'show-inheritance': True,
    'member-order': 'bysource',
}
autodoc_typehints = 'description'
autosummary_generate = True

# Napoleon settings (Google/NumPy docstring support)
napoleon_google_docstrings = True
napoleon_numpy_docstrings = True
napoleon_include_init_with_doc = True

# Intersphinx
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
}

# -- Options for HTML output
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_title = 'NX-MIMOSA Documentation'
html_short_title = 'NX-MIMOSA'

html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'sticky_navigation': True,
    'includehidden': True,
    'titles_only': False,
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'both',
}

# Custom CSS
html_css_files = []

# -- Options for LaTeX output
latex_elements = {
    'papersize': 'a4paper',
}
