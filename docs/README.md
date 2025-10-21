# Documentation Setup
This directory contains the MkDocs-based documentation for the ADG ROS2 project.

## Features
- **Static Site Generation**: Uses MkDocs with Material theme
- **API Documentation**: Automatically generates documentation from Python docstrings

## To Add
Robot Setup:
- Franka Desk logins
- SSH Instructions
- FCI screenshots



## Quick Start
### Prerequisites
```bash
# Install documentation dependencies
pip install -r docs-requirements.txt
```

### Build Documentation
```bash
# Build static site
mkdocs build

# Serve locally for development
mkdocs serve
```

The documentation will be available at `http://localhost:8000`.

### Live Development
When running `mkdocs serve`, the documentation auto-reloads when you make changes to:
- Markdown files in `docs/`
- Configuration in `mkdocs.yml`
- Python docstrings in the source code

## Documentation Structure

```
docs/
├── index.md                    # Homepage
├── user-guide/                 # User documentation
│   ├── getting-started.md
│   ├── installation.md
│   └── quick-start.md
├── packages/                   # Package-specific docs
│   ├── index.md
│   ├── adg-ros2-utils.md
│   ├── robot-arm.md
│   ├── robot-tasks.md
│   └── teleop.md
├── reference/                  # API reference
│   └── index.md
├── developer-guide/            # Developer documentation
│   ├── contributing.md
│   ├── architecture.md
│   └── testing.md
└── tutorials/                  # Step-by-step guides
    └── index.md
```

## Configuration

### MkDocs Configuration (`mkdocs.yml`)
Key configuration sections:

```yaml
# Theme and appearance
theme:
  name: material
  features:
    - navigation.tabs
    - search.suggest
    - content.code.copy

# Plugins for functionality
plugins:
  - search              # Site-wide search
  - mkdocstrings:       # API doc generation
      handlers:
        python:
          options:
            docstring_style: google

# Markdown extensions for enhanced formatting
markdown_extensions:
  - pymdownx.highlight  # Code syntax highlighting
  - pymdownx.superfences # Advanced code blocks
  - admonition          # Note/warning boxes
```

## Deployment

### GitHub Pages
The documentation can be automatically deployed to GitHub Pages:

```yaml
# .github/workflows/docs.yml
name: Deploy Documentation
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
    - uses: actions/checkout@v3
    - name: Setup Python
      uses: actions/setup-python@v4
      with:
        python-version: 3.x
    - run: pip install -r docs-requirements.txt
    - run: mkdocs gh-deploy --force
```

### Manual Deployment

```bash
# Deploy to GitHub Pages
mkdocs gh-deploy

# Build for other hosting
mkdocs build
# Upload contents of site/ directory to your web server
```

## Writing Good Documentation
### Generating API Documentation from Docstrings
MkDocs can generate documentation from Python docstrings. This setup uses:

1. **mkdocstrings**: Extracts docstrings and converts them to markdown
2. **Google-style docstrings**: Supports the Google docstring format
3. **Automatic discovery**: Finds and documents Python modules automatically

#### Example Docstring

```python
def wait_for_debugger(node_name, default_port=5678):
    """
    Wait for the debugger to attach.
    
    Only wait if the node_name matches the environment variable DEBUG_NODE.
    
    Args:
        node_name: Name of the ROS2 node for debugging
        default_port: Port to listen on for debugger connection
        
    Raises:
        ValueError: If node_name is None
        RuntimeError: If debugger fails to bind to port
        
    Example:
        >>> wait_for_debugger("my_robot_node", 5678)
        [my_robot_node] Waiting for debugger to attach on port 5678...
    """
    # Implementation here
```

#### Including Docstrings in Documentation

To include a module's docstrings in your documentation:

```markdown
# My Module Documentation

::: my_package.my_module
```

This will automatically:
- Extract all functions, classes, and methods
- Format docstrings nicely
- Include type hints
- Show inheritance relationships
- Generate cross-references


### For Python Code

1. **Use Google-style docstrings**:
   ```python
   def my_function(param1: str, param2: int = 0) -> bool:
       """
       Brief description.
       
       Args:
           param1: Description of param1
           param2: Description with default value
           
       Returns:
           Description of return value
           
       Raises:
           ValueError: When this happens
       """
   ```

2. **Include examples**:
   ```python
   """
   Example:
       >>> result = my_function("hello", 42)
       >>> print(result)
       True
   """
   ```

3. **Use type hints**: They automatically appear in the generated documentation

### For Markdown Files

1. **Use clear headings** for navigation
2. **Include code examples** with syntax highlighting
3. **Add diagrams** using Mermaid syntax:
   ```markdown
   ```mermaid
   graph TD
       A[Start] --> B[Process]
       B --> C[End]
   ```

4. **Cross-reference** other sections:
   ```markdown
   See the [API Reference](../reference/index.md) for details.
   ```

## Tips and Tricks

### Organizing Large Projects
- Use the `nav:` section in `mkdocs.yml` to control page order
- Group related content in subdirectories
- Use `index.md` files for section overviews
- Include a site-wide search

### Including Code from Source Files
```markdown
--8<-- "path/to/source/file.py:10:20"
```

This includes lines 10-20 from the specified file.

## Troubleshooting

### Common Issues

**"Module not found" when generating API docs**
- Ensure Python modules are in the correct path
- Check that `__init__.py` files exist
- Verify module imports work independently

**Styling issues**
- Check that all markdown extensions are installed
- Verify theme configuration in `mkdocs.yml`
- Clear the `site/` directory and rebuild

**Navigation problems**
- Check file paths in `nav:` configuration
- Ensure all referenced files exist
- Use relative paths consistently

### Getting Help
- [MkDocs Documentation](https://www.mkdocs.org/)
- [Material Theme Docs](https://squidfunk.github.io/mkdocs-material/)
- [mkdocstrings Documentation](https://mkdocstrings.github.io/)

