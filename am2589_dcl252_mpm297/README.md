# Webpage 

## Generating the HTML Report

To convert [index.md](./index.md) file into a styled HTML webpage:

1. Install [Pandoc](https://pandoc.org/)
2. Make the build script executable: `chmod +x build.sh`
3. Build: `./build.sh`
4. View the generated HTML file: `open index.html`

> [!WARNING]
> Do not edit `index.html` directly! It is not tracked by Git! Instead, always edit the Markdown file and generate HTML when needed.
