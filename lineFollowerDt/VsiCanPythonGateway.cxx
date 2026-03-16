#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <VsiCommon.h>
#include <map>
#include <vector>
#include "VsiCanGateway.h"

static VsiCanGateway *canGateway = NULL;
static std::map<uint8_t, VtlTlmCanConfig *> receivedCanPacketMap;

//---------------------------------------------------------
// Method:rxFrameHandler()
// the rxFrameHandler is the callback function that we register 
// in the VsiCanGateway, when it's called, we save the raw sideband 
// config of the received CAN packet. This sideband config is a pointer 
// that already contains all the data in the VtlTlmCanConfig 
// such as payload, frame ID, data length, etc. When the recvCanPacket()
// method is called, we first set this data pointer in the gateway
// using the setVtlTlmCanConfig() method, and the read the data from
// the received frame using the readCanPayloadBits() method
//---------------------------------------------------------
static void rxFrameHandler(VtlTlmCanConfig &canPacket, void *context)
{
    unsigned char* sideBandCopy;
    unsigned int frameId = canPacket.getFrameBaseMsgId();

    if (receivedCanPacketMap.count(frameId) == 0) 
    {
        receivedCanPacketMap[frameId] = new VtlTlmCanConfig;
        receivedCanPacketMap[frameId]->setDefaults();

        // To avoid pointing the same memory location when setting the sideband config
        // We will copy the raw sideband config to a new memory location and set it
        // to the receivedCanPacketMap
        sideBandCopy = new unsigned char[canPacket.get_sideband_config_num_bytes()];
        receivedCanPacketMap[frameId]->set_raw_sideband_config(sideBandCopy);
    }
    else{
        // For an existing entry, just update the sideband config to avoid new memory allocation
        sideBandCopy = receivedCanPacketMap[frameId]->get_raw_sideband_config();
    }

    // Copy the received sideband config to the sideBandCopy
    memcpy(sideBandCopy, canPacket.get_raw_sideband_config(), canPacket.get_sideband_config_num_bytes());
}

//---------------------------------------------------------
// Method:recvVariableFromCanPacket()
// the recvVariableFromCanPacket is called each step in the application's
// mainThread() for every variable that the thread reads. It takes the byte 
// length of the variable, the start bit and bit size of its location in the 
// CAN frame, and the ID of the CAN frame it will read it from, and uses the 
// readCanPayloadBits() to read the relevant data bits, and returns it along 
// with the byte length of the read data
//---------------------------------------------------------
static PyObject* recvVariableFromCanPacket(PyObject* self, PyObject* args) 
{
    uint32_t variableByteLength;
    uint8_t startBit;
    uint8_t bitSize;
    uint8_t canId;

    if(!PyArg_ParseTuple(args, "IBBB", &variableByteLength, &startBit, &bitSize, &canId))
        return NULL;
    
    // Initializes all elements to 0
    uint8_t* payload = new uint8_t[variableByteLength](); 
    if(receivedCanPacketMap.count(canId) > 0 && receivedCanPacketMap[canId]->getFrameType() == 0) 
    {
        canGateway->setVtlTlmCanConfig(*receivedCanPacketMap[canId]);
        canGateway->readCanPayloadBits(payload, startBit, bitSize, variableByteLength);
    }

    PyObject* result = Py_BuildValue("y#", payload, variableByteLength);
    delete [] payload;
    return result;
}

//---------------------------------------------------------
// Method:initialize()
// Python API that parses the RawcTlmApiThreaded instance and
// initializes the VsiCanGateway
//---------------------------------------------------------
static PyObject* initialize(PyObject* self, PyObject* args) 
{
    unsigned int conduitId;
    PyObject* capsule;

    if(!PyArg_ParseTuple(args, "Oi", &capsule, &conduitId))
        return NULL;

    RawcTlmApiThreaded* dSession = static_cast<RawcTlmApiThreaded*>(PyCapsule_GetPointer(capsule, nullptr));
    string conduitIdString = to_string(conduitId);
    string rxCanFrameConduit = string(":rxCanFrameConduit") + conduitIdString;
    string txCanFrameConduit = string(":txCanFrameConduit") + conduitIdString;
    
    canGateway = new VsiCanGateway(dSession, txCanFrameConduit.c_str(), rxCanFrameConduit.c_str());
    canGateway->registerRxCallback(NULL, rxFrameHandler);

    return Py_BuildValue("i", 0);
}

//---------------------------------------------------------
// Method:terminate()
// Python API that terminates the CAN gateway's
// connection
//---------------------------------------------------------
static PyObject* terminate(PyObject* self, PyObject* args) 
{
    int terminated = 1;
    try 
    { 
        delete canGateway;
    }
    catch (...) 
    {
        terminated = 0;
    }
    return Py_BuildValue("i", terminated);
}



//---------------------------------------------------------
// Method:destruct()
// Destructor for the Python module
//---------------------------------------------------------
void destruct(void *module) 
{
    for(std::map<uint8_t, VtlTlmCanConfig *>::iterator itr = receivedCanPacketMap.begin(); itr != receivedCanPacketMap.end(); itr++)
    {
        // Free the memory allocated for the sideband config
        delete[] itr->second->get_raw_sideband_config();
        // Free the memory allocated for the VtlTlmCanConfig itself
        delete itr->second;
    }
    receivedCanPacketMap.clear();
    delete canGateway;
}



//---------------------------------------------------------
// Method::setCanId()
// This method sets the canId
//---------------------------------------------------------
static PyObject* setCanId(PyObject* self, PyObject* args) 
{
    uint32_t canId;
    if(!PyArg_ParseTuple(args, "I", &canId))
        return NULL;
	canGateway->setCanId(canId);
    Py_RETURN_NONE;
}

//---------------------------------------------------------
// Method::setDataLengthInBits()
// This method sets the data length in bits
//---------------------------------------------------------
static PyObject* setDataLengthInBits(PyObject* self, PyObject* args) 
{
    uint8_t dataLengthInBits;
    if(!PyArg_ParseTuple(args, "B", &dataLengthInBits))
        return NULL;   
    canGateway->setDataLengthInBits(dataLengthInBits);
    Py_RETURN_NONE;
}

//---------------------------------------------------------
// Method::setCanPayloadBits()
// This method takes the data to be sent, the start bit
// and the bit size; it uses the number of bits to 
// cast the data to its appropriate type and then calls
// the setCanPayloadBits() method
//---------------------------------------------------------
static PyObject* setCanPayloadBits(PyObject* self, PyObject* args) 
{
    Py_buffer data_obj; 
    uint8_t start_bit;
    uint8_t bit_size;
    
    if (!PyArg_ParseTuple(args, "y*BB", &data_obj, &start_bit, &bit_size)) 
        return NULL;

    // Initializes all elements to 0
    uint8_t* payload = new uint8_t[data_obj.len]();
    memcpy(payload, data_obj.buf, data_obj.len);
    canGateway->setCanPayloadBits(payload, start_bit, bit_size);

    PyBuffer_Release(&data_obj);
    delete [] payload;
    Py_RETURN_NONE;
}

//---------------------------------------------------------
// Method::setCanPayloadBitsForStringArray()
// This method takes the data to be sent, the start bit
// ,the bit size and the array size; This method creates 
// a string array from the data and then calls the 
// setCanPayloadBitsForStringArray() method
//---------------------------------------------------------
static PyObject* setCanPayloadBitsForStringArray(PyObject* self, PyObject* args) 
{
    Py_buffer dataObj; 
    uint16_t startBit;
    uint16_t bitSize;
    uint16_t arraySize;
    
    if (!PyArg_ParseTuple(args, "y*HHH", &dataObj, &startBit, &bitSize, &arraySize)) 
        return NULL;
    
    // Initializes all elements to 0
    uint8_t* payload = new uint8_t[dataObj.len](); 
    memcpy(payload, dataObj.buf, dataObj.len);
    string* stringArray = new string[arraySize];
    
    uint8_t* currentPayload = payload;
    for (int i = 0; i < arraySize; i++)
    {
        stringArray[i] = string((char*)currentPayload);
        currentPayload += stringArray[i].size() + 1;
    }
    
    canGateway->setCanPayloadBitsForStringArray(stringArray, arraySize, startBit, bitSize);

    PyBuffer_Release(&dataObj);
    delete [] stringArray;
    delete [] payload;
    Py_RETURN_NONE;
}

//---------------------------------------------------------
// Method::setCanPayloadBitsForString()
// This method takes the data to be sent, the start bit and the bit size;
// This method creates a string from the data and then calls the 
// setCanPayloadBitsForString() method
//---------------------------------------------------------
static PyObject* setCanPayloadBitsForString(PyObject* self, PyObject* args) 
{
    Py_buffer dataObj; 
    uint16_t startBit;
    uint16_t bitSize;
    
    if (!PyArg_ParseTuple(args, "y*HH", &dataObj, &startBit, &bitSize)) 
        return NULL;

    // Initializes all elements to 0
    uint8_t* payload = new uint8_t[dataObj.len](); 
    memcpy(payload, dataObj.buf, dataObj.len);
    string stringData = string((char*)payload);
    canGateway->setCanPayloadBits(stringData, startBit, bitSize);

    PyBuffer_Release(&dataObj);
    delete [] payload;
    Py_RETURN_NONE;
}

//---------------------------------------------------------
// Method::sendCanPacket()
// This method sends the can packet, it should be called
// after using both setCanId() and setCanPayloadBits()
//---------------------------------------------------------
static PyObject* sendCanPacket(PyObject* self, PyObject* args) 
{
    canGateway->sendCanPacket();
    return Py_BuildValue("i", 1);
}

//---------------------------------------------------------
// The module's function definition struct
//---------------------------------------------------------
static PyMethodDef clientMethods[] = 
{
    { "initialize", initialize, METH_VARARGS, "Connect to TLM fabric server" },
    { "terminate", terminate, METH_VARARGS},
    { "recvVariableFromCanPacket", recvVariableFromCanPacket, METH_VARARGS, "Reads a variable from CAN packets received from TLM fabric server" },
    { "sendCanPacket", sendCanPacket, METH_VARARGS, "Send CAN packets to TLM fabric server" },
    { "setCanId", setCanId, METH_VARARGS},
    { "setDataLengthInBits", setDataLengthInBits, METH_VARARGS},
    { "setCanPayloadBits", setCanPayloadBits, METH_VARARGS},
    { "setCanPayloadBitsForStringArray", setCanPayloadBitsForStringArray, METH_VARARGS},
    { "setCanPayloadBitsForString", setCanPayloadBitsForString, METH_VARARGS},
    { NULL, NULL, 0, NULL }
};

static struct PyModuleDef VsiCanPythonGateway = 
{
    PyModuleDef_HEAD_INIT,
    "VsiCanPythonGateway",
    "C Client Module",
    -1,
    clientMethods,
    NULL,
    NULL,
    NULL,
    destruct
};

PyMODINIT_FUNC PyInit_VsiCanPythonGateway(void) 
{
    return PyModule_Create(&VsiCanPythonGateway);
}
