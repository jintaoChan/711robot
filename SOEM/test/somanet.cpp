/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage : slaveinfo [ifname] [-sdo] [-map]
 * Ifname is NIC interface, f.e. eth0.
 * Optional -sdo to display CoE object dictionary.
 * Optional -map to display slave PDO mapping
 *
 * This shows the configured slave data.
 *
 * (c)Arthur Ketels 2010 - 2011
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#define _WINSOCKAPI_
#include <Windows.h>

#include "ethercat.h"

char IOmap[4096];


/* define pointer structure */
#pragma pack(1)
typedef struct {
    uint16_t Statusword;
    int8_t OpModeDisplay;
    int32_t PositionValue;
    int32_t VelocityValue;
    int16_t TorqueValue;
    uint16_t AnalogInput1;
    uint16_t AnalogInput2;
    uint16_t AnalogInput3;
    uint16_t AnalogInput4;
    uint32_t TuningStatus;
    uint32_t DigitalInputs;
    uint32_t UserMISO;
    uint32_t Timestamp;
    int32_t PositionDemandInternalValue;
    int32_t VelocityDemandValue;
    int16_t TorqueDemand;
} in_somanet_50t;

typedef struct {
    uint16_t Controlword;
    int8_t OpMode;
    int16_t TargetTorque;
    int32_t TargetPosition;
    int32_t TargetVelocity;
    int16_t TorqueOffset;
    int32_t TuningCommand;
    int32_t PhysicalOutputs;
    int32_t BitMask;
    int32_t UserMOSI;
    int32_t VelocityOffset;
} out_somanet_50t;
#pragma pack()


struct SlaveIO
{
    in_somanet_50t* input;
    out_somanet_50t* output;
};


void CALLBACK RTthread(UINT uTimerID, UINT uMsg, DWORD_PTR dwUser, DWORD_PTR dw1, DWORD_PTR dw2)
{

    while (1)
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);



        //printf("control_word: %x\tstatus_word: %x\n", slave_io.output->control, slave_io.input->status);
        //printf("target_position: %d\ttarget_velocity: %d\n", slave_io.output->position, slave_io.output->velocity);

    }


}


void slaveinfo(const char* ifname)
{
    UINT mmResult;

    printf("Starting slaveinfo\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n", ifname);
        /* find and auto-config slaves */
        if (ec_config_init(FALSE) > 0)
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            //ec_slave[1].PO2SOconfig = &RDMsetup;

            ec_config_map(&IOmap);

            ec_configdc();

            printf("Slaves mapped, state to SAFE_OP.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);


            /* request OP state for all slaves */
            ec_writestate(0);

            int chk = 200;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
            ec_readstate();//读取状态


            if (ec_slave[1].state == EC_STATE_OPERATIONAL) {
                printf("state to OPERATION.\n");
            }
            else {
                printf("ERROR! \n");
                return;
            }

            //mmResult = timeSetEvent(1, 0, RTthread, 0, TIME_PERIODIC);

            SlaveIO slave_io;

            slave_io.output = (out_somanet_50t*)ec_slave[1].outputs;
            slave_io.input = (in_somanet_50t*)ec_slave[1].inputs;
            while (1) {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                slave_io.output->OpMode = 8;

                printf("position: %d\tvelocity: %d\ttorque: %d\tcontrol_status: %x\n", slave_io.input->PositionValue, slave_io.input->VelocityValue, slave_io.input->TorqueValue, slave_io.input->Statusword);
                //printf("Current position: %d\tTarget Position %d\tcontrol_status: %x\n", position_actual_value, target_position, control_status);

                if ((slave_io.input->Statusword & 0b0000000001001111) == 0b0000000000001000)
                {
                    slave_io.output->Controlword = 0b10000000;
                }
                else if ((slave_io.input->Statusword & 0b0000000001001111) == 0b0000000001000000)
                {
                    slave_io.output->Controlword = 0b00000110;
                }
                else if ((slave_io.input->Statusword & 0b0000000001101111) == 0b0000000000100001)
                {
                    slave_io.output->Controlword = 0b00000111;
                }
                else if ((slave_io.input->Statusword & 0b0000000001101111) == 0b0000000000100011)
                {
                    slave_io.output->Controlword = 0b00001111;
                }
                else if ((slave_io.input->Statusword & 0b0000000001101111) == 0b0000000000100111)
                {
                    slave_io.output->TargetPosition = slave_io.input->PositionValue - 3;
                }
                printf("position: %d\tvelocity: %d\ttorque: %d\tcontrol_status: %x\n", slave_io.input->PositionValue, slave_io.input->VelocityValue, slave_io.input->TorqueValue, slave_io.input->Statusword);

            }

            //timeKillEvent(mmResult);


        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End slaveinfo, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

char ifbuf[1024];

int main(int argc, char* argv[])
{
    printf("SOEM (Simple Open EtherCAT Master)\nSlaveinfo\n");
    slaveinfo("/Device/NPF_{C7192FE4-D3BE-49DE-88F2-BFA7AE789A19}");

    printf("End program\n");
    return (0);
}
