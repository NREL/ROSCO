# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
from datetime import date
root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, root_path)
sys.path.insert(0, os.path.join(root_path,'Examples'))

# -- Project information -----------------------------------------------------

project = 'ROSCO'
copyright = '2021, NREL'
author = 'Nikhar J. Abbas, Daniel S. Zalkind'

# The full version, including alpha/beta/rc tags
import rosco.toolbox
release = str(rosco.toolbox.__version__)

from unittest import mock
class MockModule(mock.Mock):
    @classmethod
    def __getattr__(cls, name):
        return MockModule()


MOCK_MODULES = ("ntlm",)

sys.modules.update((mod_name, MockModule()) for mod_name in MOCK_MODULES)

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.mathjax",
    "sphinx.ext.viewcode",
    "sphinx.ext.githubpages",
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "sphinx.ext.graphviz",
    "sphinx.ext.autosectionlabel",
    "sphinx_rtd_theme",
    "sphinx.ext.autodoc",
    # "sphinxcontrib.bibtex",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This patterns also effect to html_static_path and html_extra_path
# exclude_patterns = ["_build", ".DS_Store", "_templates"]

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = "friendly"

# # The master toctree document.
master_doc = "index"

# # If true, `todo` and `todoList` produce output, else they produce nothing.
# todo_include_todos = False

# enable numref
numfig = True

# General information about the project.
project = "ROSCO"
copyright = f"{date.today().year}, National Renewable Energy Laboratory"

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ['_static']

# Options for AutoDoc:
add_module_names = False
