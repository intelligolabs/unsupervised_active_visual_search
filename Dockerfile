# Use the ubuntu:18.04 base image
FROM ubuntu:18.04

# Install make, libbost 1.65.1 and build-essential
RUN apt-get update && \
    apt-get install -y make && \
    apt-get install build-essential -y && \
    apt-get install libboost-all-dev -y && \
    apt-get install -y cmake

# Set the default command to be executed when the container starts
CMD ["/bin/bash"]