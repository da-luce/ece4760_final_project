#!/bin/sh

# Convert index.md to index.html using Pandoc

# Allow Pandoc to look for resources (like images) in the current directory
# Embed all linked resources (images, etc.)
# Generate a standalone HTML file
# Link external CSS file
# Enable MathJax support for rendering LaTeX-style math expressions
# Generate a table of contents
# Apply Zenburn code syntax highlighting
# Highlight "TODO:" comments in the text

pandoc index.md -o index.html \
    --resource-path=. \
    --embed-resources \
    --standalone \
    --css=styles.css \
    --mathjax \
    --toc \
    --highlight-style=zenburn \
    --lua-filter=highlight-todo.lua \
    --citeproc