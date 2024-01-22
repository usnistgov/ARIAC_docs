# Instructions to build documentation locally (Ubuntu)

1. Install Sphinx

    ```
    sudo apt-get install python3-sphinx
    ```

2. Install python virtual environments

    ```
    sudo apt-get install python3-venv
    ```

3. If running on wsl, this step is needed to be able to browse using the command line

    ```
    sudo apt install wslu
    export BROWSER=wslview
    sudo apt install xdg-utils
    ```

3. Create a virtual environment for building the sphinx project

    ```
    python3 -m venv .ariac_docs
    ```

4. Activate the virtual environment

    ```
    source .ariac_docs/bin/activate
    ```

5. Clone the documentation repository 

    ```
    git clone https://github.com/usnistgov/ARIAC_docs.git
    ```

6. Install the necessary sphinx extensions

    ```
    cd ARIAC_docs
    pip install -r requirements.txt
    ```

7. Make the html 

    ```
    cd docs
    make html
    ```

8. Open the documentation in a browser 

    ```
    browse _build/html/index.html
    ```