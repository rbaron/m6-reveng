FROM ubuntu:20.10

RUN apt-get update && \
	apt-get install -y wget make bzip2 unzip

RUN cd /opt && \
	wget http://shyboy.oss-cn-shenzhen.aliyuncs.com/readonly/tc32_gcc_v2.0.tar.bz2 && \
	tar jxvf tc32_gcc_v2.0.tar.bz2

RUN cd /opt && \
	wget http://wiki.telink-semi.cn/tools_and_sdk/BLE/8232_BLE_SDK.zip && \
	unzip 8232_BLE_SDK.zip

WORKDIR /app

ENV SDK="/opt/8232_BLE_SDK/ble_sdk_hawk" \
	TC32_HOME="/opt/tc32" \
	PATH="/opt/tc32/bin:${PATH}"