# Documentation Setup
This directory contains the [Zensical](https://zensical.org/docs/get-started/)-based documentation for the ADL ROS2 project.

**Doc To Add**
Robot Setup:
- [ ] Franka Desk logins
- [ ] SSH Instructions
- [ ] FCI screenshots


### Build Documentation

```bash
zensical serve
```

The documentation will be available at `http://localhost:8000`. The documentation auto-reloads when you make changes.


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

## Tips for Writing Good Documentation
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


### Including Code from Source Files
```markdown
--8<-- "path/to/source/file.py:10:20"
```

This includes lines 10-20 from the specified file.


## Getting Help
- [Zensical Documentation](https://zensical.org/docs)

