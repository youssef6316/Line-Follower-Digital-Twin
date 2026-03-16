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
		self.sim_time = 0
		self.path_progress = 0

		# Outputs
		self.v_cmd = 0
		self.omega_cmd = 0
		self.e_lat = 0
		self.e_head = 0


# Start of user custom code region: Global Variables & Definitions
import os
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from path   import make_straight, make_s_curve
from errors import compute_errors
from pid    import PIDController, GAINS_PID

PATH_TYPE  = "straight"   # must match simulator
CONTROL_DT = 0.05         # 20 Hz
# End of user custom code region.


class Controller:

	def __init__(self, args):
		self.componentId = 1
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50102

		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0

		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region: Constructor
		self.path = make_straight(15.0) if PATH_TYPE == "straight" else make_s_curve()
		self.pid  = PIDController(GAINS_PID, dt=CONTROL_DT)

		# nextExpectedTime gates the 20 Hz controller inside the 100 Hz loop
		self.nextExpectedTime = 0.0

		# Initialise output signals so controller sends valid data from step 0
		self.mySignals.v_cmd     = GAINS_PID.v_ref
		self.mySignals.omega_cmd = 0.0
		self.mySignals.e_lat     = 0.0
		self.mySignals.e_head    = 0.0
		print(f"[Controller] Initialised with gains: {GAINS_PID.name}")
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

				# VSI auto-generated: receive pose signals
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
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 16)
				self.mySignals.sim_time, receivedData = self.unpackBytes('d', receivedData, self.mySignals.sim_time)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 17)
				self.mySignals.path_progress, receivedData = self.unpackBytes('d', receivedData, self.mySignals.path_progress)

				# Start of user custom code region: Before sending the packet
				# 20 Hz gate: only run PID every CONTROL_DT seconds
				current_time = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
				if current_time >= self.nextExpectedTime:
					# Compute errors from current pose
					nr   = self.path.get_nearest(self.mySignals.x, self.mySignals.y)
					errs = compute_errors(
						self.mySignals.x,
						self.mySignals.y,
						self.mySignals.theta,
						nr
					)
					# Run PID
					v_cmd, omega_cmd = self.pid.step(errs.e_lat, errs.e_head)

					# Update output signals
					self.mySignals.v_cmd     = v_cmd
					self.mySignals.omega_cmd = omega_cmd
					self.mySignals.e_lat     = errs.e_lat
					self.mySignals.e_head    = errs.e_head

					# Advance gate to next 20 Hz tick
					self.nextExpectedTime += CONTROL_DT
				# End of user custom code region.

				# VSI auto-generated: send output signals
				vsiCanPythonGateway.setCanId(18)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.v_cmd), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(19)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.omega_cmd), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(20)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.e_lat), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(21)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.e_head), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

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
	controller = Controller(args)
	controller.mainThread()

if __name__ == '__main__':
	main()