FROM ubuntu:20.04

ARG EXEC_NAME
ENV EXEC_NAME=$EXEC_NAME

COPY $EXEC_NAME /

CMD ["sh", "-c", "/$EXEC_NAME"]

