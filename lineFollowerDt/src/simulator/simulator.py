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
		self.v_cmd = 0
		self.omega_cmd = 0

		# Outputs
		self.x = 0
		self.y = 0
		self.theta = 0
		self.x_true = 0
		self.y_true = 0
		self.theta_true = 0
		self.sim_time = 0
		self.path_progress = 0


# Start of user custom code region: Global Variables & Definitions
import os
import numpy as np
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from robot import DifferentialDriveRobot
from path  import make_straight, make_s_curve, make_random_spawn

PATH_TYPE   = "straight"   # change to "scurve" for E2/E4
NOISE_POS   = 0.01
NOISE_THETA = 0.005
RNG_SEED    = 42
# End of user custom code region.


class Simulator:

	def __init__(self, args):
		self.componentId = 0
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50101

		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0

		self.receivedNumberOfBytes = 0
		self.receivedPayload = []
		self.mySignals = MySignals()

		# Start of user custom code region: Constructor
		_path = make_straight(15.0) if PATH_TYPE == "straight" else make_s_curve()
		self.path = _path
		_rng = np.random.default_rng(RNG_SEED)
		_x0, _y0, _t0 = make_random_spawn(self.path, lateral_range=1.2,
		                                   heading_noise_deg=15, rng=_rng)
		self.robot = DifferentialDriveRobot(
			x0=_x0, y0=_y0, theta0=_t0,
			dt=0.01,
			noise_std_pos=NOISE_POS,
			noise_std_theta=NOISE_THETA,
		)
		print(f"[Simulator] Spawned at x={_x0:.3f} y={_y0:.3f} theta={np.rad2deg(_t0):.1f}deg")
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

				# VSI auto-generated: receive v_cmd, omega_cmd
				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 18)
				self.mySignals.v_cmd, receivedData = self.unpackBytes('d', receivedData, self.mySignals.v_cmd)

				signalNumBytes = 8
				receivedData = vsiCanPythonGateway.recvVariableFromCanPacket(signalNumBytes, 0, 64, 19)
				self.mySignals.omega_cmd, receivedData = self.unpackBytes('d', receivedData, self.mySignals.omega_cmd)

				# Start of user custom code region: Before sending the packet
				# Advance robot physics one step using received commands
				pose = self.robot.step(self.mySignals.v_cmd, self.mySignals.omega_cmd)

				# Compute path progress
				nr = self.path.get_nearest(pose["x"], pose["y"])

				# Set all output signals
				self.mySignals.x           = pose["x"]
				self.mySignals.y           = pose["y"]
				self.mySignals.theta       = pose["theta"]
				self.mySignals.x_true      = pose["x_true"]
				self.mySignals.y_true      = pose["y_true"]
				self.mySignals.theta_true  = pose["theta_true"]
				self.mySignals.sim_time    = vsiCommonPythonApi.getSimulationTimeInNs() * 1e-9
				self.mySignals.path_progress = nr.progress
				# End of user custom code region.

				# VSI auto-generated: send all outputs
				vsiCanPythonGateway.setCanId(10)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.x), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(11)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.y), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(12)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.theta), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(13)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.x_true), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(14)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.y_true), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(15)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.theta_true), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(16)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.sim_time), 0, 64)
				vsiCanPythonGateway.setDataLengthInBits(64)
				vsiCanPythonGateway.sendCanPacket()

				vsiCanPythonGateway.setCanId(17)
				vsiCanPythonGateway.setCanPayloadBits(self.packBytes('d', self.mySignals.path_progress), 0, 64)
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
	simulator = Simulator(args)
	simulator.mainThread()

if __name__ == '__main__':
	main()