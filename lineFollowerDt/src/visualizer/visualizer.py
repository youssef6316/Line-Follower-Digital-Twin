#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiCanPythonGateway as vsiCanPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.x = 0
		self.y = 0
		self.theta = 0
		self.x_true = 0
		self.y_true = 0
		self.theta_true = 0
		self.sim_time = 0
		self.path_progress = 0
		self.v_cmd = 0
		self.omega_cmd = 0
		self.e_lat = 0
		self.e_head = 0


# Start of user custom code region: Global Variables & Definitions
import os
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("TkAgg")   # explicit backend for DCV Linux desktop

from path              import make_straight, make_s_curve
from logger            import ExperimentLogger
from visualizer_client import VisualizerClient

PATH_TYPE = "straight"   # must match simulator
VIZ_DT    = 0.05         # 20 Hz update rate
# End of user custom code region.


class Visualizer:

	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103

		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0

		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region: Constructor
		self.path       = make_straight(15.0) if PATH_TYPE == "straight" else make_s_curve()
		self.exp_logger = ExperimentLogger("vsi_run", results_dir="results")
		self.viz        = VisualizerClient(
			path            = self.path,
			gains_name      = "G3_baseline",
			exp_logger      = self.exp_logger,
			live_plot       = True,
			results_dir     = "results",
			experiment_name = "vsi_run",
		)
		# 20 Hz gate
		self.nextExpectedTime = 0.0
		print("[Visualizer] Initialised — live plot window will open")
		# End of user custom code region.


	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiCanPythonGateway.initialize(dSession, self.componentId)
		try:
			vsiCommonPythonApi.waitForReset()
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region: Inside the while loop
				# (nothing needed here — inputs arrive below, logic goes in Before sending)
				# End of user custom code region.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				# VSI auto-generated: receive all signals
				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 10)
				self.mySignals.x, receivedData = self.unpackBytes('d', receivedData, self.mySignals.x)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 11)
				self.mySignals.y, receivedData = self.unpackBytes('d', receivedData, self.mySignals.y)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 12)
				self.mySignals.theta, receivedData = self.unpackBytes('d', receivedData, self.mySignals.theta)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 13)
				self.mySignals.x_true, receivedData = self.unpackBytes('d', receivedData, self.mySignals.x_true)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 14)
				self.mySignals.y_true, receivedData = self.unpackBytes('d', receivedData, self.mySignals.y_true)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 15)
				self.mySignals.theta_true, receivedData = self.unpackBytes('d', receivedData, self.mySignals.theta_true)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 16)
				self.mySignals.sim_time, receivedData = self.unpackBytes('d', receivedData, self.mySignals.sim_time)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 17)
				self.mySignals.path_progress, receivedData = self.unpackBytes('d', receivedData, self.mySignals.path_progress)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 18)
				self.mySignals.v_cmd, receivedData = self.unpackBytes('d', receivedData, self.mySignals.v_cmd)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 19)
				self.mySignals.omega_cmd, receivedData = self.unpackBytes('d', receivedData, self.mySignals.omega_cmd)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 20)
				self.mySignals.e_lat, receivedData = self.unpackBytes('d', receivedData, self.mySignals.e_lat)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 21)
				self.mySignals.e_head, receivedData = self.unpackBytes('d', receivedData, self.mySignals.e_head)

				# Start of user custom code region: Before sending the packet
				# 20 Hz gate: update visualizer at reduced rate
				current_time = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
				if current_time >= self.nextExpectedTime:
					self.viz.update(
						t           = current_time,
						x           = self.mySignals.x,
						y           = self.mySignals.y,
						theta       = self.mySignals.theta,
						x_true      = self.mySignals.x_true,
						y_true      = self.mySignals.y_true,
						theta_true  = self.mySignals.theta_true,
						v           = self.mySignals.v_cmd,
						omega       = self.mySignals.omega_cmd,
						e_lat       = self.mySignals.e_lat,
						e_head      = self.mySignals.e_head,
						path_progress = self.mySignals.path_progress,
					)
					self.nextExpectedTime += VIZ_DT
				# End of user custom code region.

				# Visualizer has no output signals to send

				# Start of user custom code region: After sending the packet
				# End of user custom code region.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal has been received from one of the VSI clients")
				# Finalise KPIs before exit
				try:
					self.viz.finalize()
					self.exp_logger.finalize()
				except Exception:
					pass
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)


	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			if signalType == 's':
				packedData = b''
				for str in signal:
					str += '\0'
					str = str.encode('utf-8')
					packedData += struct.pack(f'={len(str)}s', str)
				return packedData
			else:
				return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			if signalType == 's':
				signal += '\0'
				signal = signal.encode('utf-8')
				return struct.pack(f'={len(signal)}s', signal)
			else:
				return struct.pack(f'={signalType}', signal)

	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			if signalType == 's':
				unpackedStrings = [''] * len(signal)
				for i in range(len(signal)):
					nullCharacterIndex = packedBytes.find(b'\0')
					if nullCharacterIndex == -1:
						break
					unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
					unpackedStrings[i] = unpackedString
					packedBytes = packedBytes[nullCharacterIndex + 1:]
				return unpackedStrings, packedBytes
			else:
				unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
				packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
				return list(unpackedVariable), packedBytes
		elif signalType == 's':
			nullCharacterIndex = packedBytes.find(b'\0')
			unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
			packedBytes = packedBytes[nullCharacterIndex + 1:]
			return unpackedVariable, packedBytes
		else:
			numBytes = 0
			if signalType in ['?', 'b', 'B']:
				numBytes = 1
			elif signalType in ['h', 'H']:
				numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']:
				numBytes = 4
			elif signalType in ['q', 'Q', 'd']:
				numBytes = 8
			else:
				raise Exception('received an invalid signal type in unpackBytes()')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()


def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL')
	args = inputArgs.parse_args()
	visualizer = Visualizer(args)
	visualizer.mainThread()

if __name__ == '__main__':
	main()