# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ARIAC Docs'
version = '2025.1.0'
copyright = u'works of NIST employees are not not subject to copyright protection in the United States'
author = 'Justin Albrecht'

html_theme = 'furo'

html_static_path = ["_static"]

html_theme_options = {
    "light_logo": "nist_el_logo_color.png",
    "dark_logo": "nist_el_logo_white.png",
    "sidebar_hide_name": True,
}

templates_path = ['_templates']

extensions = [
    'sphinx.ext.mathjax',
    'sphinx.ext.todo',
    'sphinx_copybutton',
    'sphinxemoji.sphinxemoji'
]