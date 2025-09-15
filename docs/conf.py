# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ARIAC Docs'
version = '2025.1.0'
copyright = u'works of NIST employees are not not subject to copyright protection in the United States'
author = 'NIST'

html_theme = 'sphinx_book_theme'

html_logo = "_static/nist_el_logo_color.png"

html_static_path = ["_static"]
html_css_files = ["custom.css"]

html_theme_options = {
    "logo": {
        "image_light": "_static/nist_el_logo_color.png",
        "image_dark": "_static/nist_el_logo_white.png",
    },
    "use_download_button": False,
    "use_fullscreen_button": False,
    
}

templates_path = ['_templates']

extensions = [
    'sphinx.ext.mathjax',
    'sphinx.ext.todo',
    'sphinx_copybutton',
    'sphinxemoji.sphinxemoji'
]