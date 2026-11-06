#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
import time
import os
import rclpy
import socket
import fcntl
import struct
from collections import deque


def cl_black(msge): return '\033[30m' + msge + '\033[0m'
def cl_red(msge): return '\033[31m' + msge + '\033[0m'
def cl_green(msge): return '\033[32m' + msge + '\033[0m'
def cl_orange(msge): return '\033[33m' + msge + '\033[0m'
def cl_blue(msge): return '\033[34m' + msge + '\033[0m'
def cl_purple(msge): return '\033[35m' + msge + '\033[0m'
def cl_cyan(msge): return '\033[36m' + msge + '\033[0m'
def cl_lightgrey(msge): return '\033[37m' + msge + '\033[0m'
def cl_darkgrey(msge): return '\033[90m' + msge + '\033[0m'
def cl_lightred(msge): return '\033[91m' + msge + '\033[0m'
def cl_lightgreen(msge): return '\033[92m' + msge + '\033[0m'
def cl_yellow(msge): return '\033[93m' + msge + '\033[0m'
def cl_lightblue(msge): return '\033[94m' + msge + '\033[0m'
def cl_pink(msge): return '\033[95m' + msge + '\033[0m'
def cl_lightcyan(msge): return '\033[96m' + msge + '\033[0m'


class TCPSocket:
    # Linux ioctl: bytes still waiting in the kernel send queue.
    SIOCOUTQ = 0x5411
    # Keep the outbound queue short so a backed-up Sunrise app cannot make
    # the arm replay several seconds of stale targets after a stop.
    MAX_SEND_QUEUE_BYTES = 4096

    def __init__(self, ip, port,node):
        self.BUFFER_SIZE = 4000
        self.isconnected = False
        self.node_name = node
        self.ip = ip
        self.port = port
        self.tcp = None
        self.connection = None

        #Data
        self.odometry = []
        # Only the newest sample matters for these streams.  Bounding the
        # queues prevents an ever-growing backlog (and O(n) pop(0) calls) if a
        # consumer is slower than the 1 kHz Sunrise stream.
        self.laserScanB1 = deque(maxlen=1)
        self.laserScanB4 = deque(maxlen=1)
        self.kmp_statusdata = None
        self.lbr_statusdata = None
        self.lbr_sensordata = deque(maxlen=1)

        threading.Thread(target=self.connect_to_socket, daemon=True).start()

    def close(self):
        self.isconnected = False

    def connect_to_socket(self):
        print(cl_cyan('Starting up node:'), self.node_name, 'IP:', self.ip, 'Port:', self.port)
        try:
            self.tcp = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_address= (self.ip,self.port)
            self.tcp.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
            self.tcp.bind(server_address)
        except:
           print(cl_red('Error: ') + "Connection for KUKA cannot assign requested address:", self.ip, self.port)

        self.tcp.listen(3)
        while (not self.isconnected):
            try:
                self.connection, client_address = self.tcp.accept()
                self.connection.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.tcp.settimeout(0.01)
                self.isconnected = True
            except:
                t=0
        time.sleep(1) 

        count = 0
        while self.isconnected:
            try:
                last_read_time = time.time()  # Keep received time
                data = self.recvmsg()
                for pack in (data.decode("utf-8")).split(">"):  # parsing data pack
                    cmd_splt = pack.split()
                    if len(cmd_splt) and cmd_splt[0] == 'odometry':
                        self.odometry = cmd_splt
                        #print('odom')
                    if len(cmd_splt) and cmd_splt[0] == 'laserScan':
                        if cmd_splt[2] == '1801':
                            self.laserScanB1.append(cmd_splt)
                            #print(count)
                            count = count + 1

                        elif cmd_splt[2] == '1802':
                            self.laserScanB4.append(cmd_splt)
                            count = count + 1
                    if len(cmd_splt) and cmd_splt[0] == 'kmp_statusdata':
                        self.kmp_statusdata = cmd_splt
                    if len(cmd_splt) and cmd_splt[0] == 'lbr_statusdata':
                        self.lbr_statusdata = cmd_splt
                    if len(cmd_splt) and cmd_splt[0] == 'lbr_sensordata':
                        self.lbr_sensordata.append(cmd_splt)

            except (ConnectionError, OSError) as e:
                # Peer closed the socket (or it broke): exit instead of
                # spinning on a dead connection.
                print(cl_red('Connection lost: ') + str(e))
                break
            except Exception as e:
                print("ERROR:", e)

        print("SHUTTING DOWN")
        self.isconnected = False
        try:
            self.connection.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            self.connection.close()
        except Exception:
            pass
        try:
            self.tcp.close()
        except Exception:
            pass
        print(cl_lightred('Connection is closed!'))
        rclpy.shutdown()


    def send_queue_bytes(self):
        """Bytes currently queued in the kernel for this connection."""
        try:
            buf = fcntl.ioctl(
                self.connection.fileno(), self.SIOCOUTQ, struct.pack('i', 0))
            return struct.unpack('i', buf)[0]
        except Exception:
            return 0

    def send(self, cmd, drop_if_busy=True):
        # Never block the ROS executor for streamed targets. If the peer is
        # behind, drop this (stale) command; the caller resends only the
        # newest target. Control messages (CancelMotion, trajectories) set
        # drop_if_busy=False because they must arrive intact.
        if self.connection is None:
            return False
        try:
            if drop_if_busy and self.send_queue_bytes() > self.MAX_SEND_QUEUE_BYTES:
                return False
            self.connection.sendall((cmd + '\r\n').encode("UTF-8"))
            return True
        except Exception as e:
            print(cl_red('Error: ') + f"sending message thread failed: {e}")
            return False

    def recvmsg(self):
        # Framing from the Sunrise app: 10 ASCII digits with the byte length
        # of the payload, then a space, then exactly that many bytes.  TCP is a
        # stream, so every read must consume the whole frame; otherwise the
        # next header is taken from payload bytes and the stream desyncs.
        header_len = 10

        byt_len = b""
        while len(byt_len) < header_len:
            chunk = self.connection.recv(header_len - len(byt_len))
            if not chunk:
                raise ConnectionError("connection closed while reading header")
            byt_len += chunk

        msglength = int(byt_len.decode("utf-8")) + 1   # include the space
        if msglength <= 0:
            return b""

        msg = b""
        while len(msg) < msglength:
            chunk = self.connection.recv(msglength - len(msg))
            if not chunk:
                raise ConnectionError("connection closed while reading payload")
            msg += chunk
        return msg
