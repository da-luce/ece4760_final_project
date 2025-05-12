# Webpage 

## Generating the HTML Report

To convert [index.md](./index.md) file into a styled HTML webpage, use [Pandoc](https://pandoc.org/):

```bash
pandoc index.md -o index.html --standalone --css=styles.css --mathjax
```

> [!WARNING]
> Do not edit `index.html` directly! It is not tracked by Git! Instead, always edit the Markdown file and generate HTML when needed.