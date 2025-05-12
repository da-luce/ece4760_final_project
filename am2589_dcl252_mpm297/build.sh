#!/bin/sh

# Convert index.md to index.html using Pandoc
pandoc index.md -o index.html \
    --resource-path=. \
    --embed-resources \
    --standalone \
    --css=styles.css \
    --mathjax \
    --toc \
    --highlight-style=zenburn \
    --lua-filter=highlight-todo.lua