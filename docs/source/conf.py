# NX-MIMOSA v6.0.1 Sphinx Configuration
import os, sys
sys.path.insert(0, os.path.abspath('../..'))

project = 'NX-MIMOSA'
copyright = '2026, Nexellum d.o.o.'
author = 'Dr. Mladen Mešter'
release = '6.0.1'
version = '6.0'

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

autodoc_default_options = {
    'members': True,
    'undoc-members': False,
    'show-inheritance': True,
    'member-order': 'bysource',
}
autodoc_typehints = 'description'
autosummary_generate = True

napoleon_google_docstrings = True
napoleon_numpy_docstrings = True
napoleon_include_init_with_doc = True

intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'numpy': ('https://numpy.org/doc/stable/', None),
    'scipy': ('https://docs.scipy.org/doc/scipy/', None),
}

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_title = 'NX-MIMOSA v5.9.3 Documentation'
html_short_title = 'NX-MIMOSA'

html_theme_options = {
    'navigation_depth': 4,
    'collapse_navigation': False,
    'sticky_navigation': True,
    # 'display_version': True,  # removed: deprecated in sphinx-rtd-theme 3.0+
    'prev_next_buttons_location': 'both',
}
