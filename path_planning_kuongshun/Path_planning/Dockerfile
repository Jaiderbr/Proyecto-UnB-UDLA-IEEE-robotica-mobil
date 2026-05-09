FROM python:3.10

WORKDIR /app

RUN pip install numpy matplotlib


RUN apt update
RUN apt install nano


COPY . /app

CMD ["python", "path_tracking_Pioneer_4W_Docker.py"]