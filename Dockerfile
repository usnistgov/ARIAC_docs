# Using the Ubuntu image (our OS)
FROM ubuntu:latest

COPY requirements.txt .

# Update package manager (apt-get) 
# and install (with the yes flag `-y`)
# Python and Pip
RUN apt-get update && \
    apt-get install -y python3 python3-pip && \
    pip install -r requirements.txt --break-system-packages