/*
 * ospfnode.h
 *
 * CS456 A2 OSPFNode Header
 * Written by Andrew J Klamut (ajklamut)
 *
 * July 2012
 *
 * This file has the OSPFNode class interface.
 */

#ifndef OSPFNODE_H
#define OSPFNODE_H

#include "common.h"
#include <map>
#include <vector>

struct SocketError {
    std::string msg;
    SocketError(std::string msg) : msg(msg) {}
};

class OSPFNode {
    // Each OSPFNode will (eventually) have a TopologyEntry for each other OSPFNode
    struct TopologyEntry {
        // Maps link ids attached to this particular OSPFNode to their costs
        std::map<unsigned int, unsigned int> link_costs;
    };
    // Each OSPFNode will (eventually) have an RIBEntry for each other OSPFNode
    struct RIBEntry {
        // Maps neighbours for this particular OSPFNode to their link costs
        std::map<unsigned int, unsigned int> neighbour_costs;
        // Represents the minimum cost of the host OSPFNode to this particular OSPFNode
        unsigned int next_hop, min_cost;
    };

    int id, local_port, remote_port, remote_fd;
    std::string remote_address;
    DebugStream debug, output;
    std::map<unsigned int, unsigned int> circuit_db;
    std::map<unsigned int, TopologyEntry *> topology_db;
    std::map<unsigned int, RIBEntry *> rib;
    std::vector<unsigned int> active_links;

    void establishConnection() throw (SocketError);
    int sendMessage(char *buffer, unsigned int length) throw (SocketError);
    int recvMessage(char *buffer, unsigned int length) throw (SocketError);
    void sendInitPacket();
    void getCircuitDatabase();
    void printCircuitDatabase();
    void sendHelloPacket(unsigned int link_id);
    void addTopologyEntry(unsigned int router_id, unsigned int link_id, unsigned int cost);
    void printTopologyDatabase();
    void sendLSPDUPacket(unsigned int sender, unsigned int router_id, unsigned int link_id, unsigned int cost, unsigned int via);
    void rebuildRIB();
    void printRIB();
    int getNeighbourId(unsigned int router_id, unsigned int link_id);
    void forwardLSPDUPacket(char *buffer, unsigned int received_link);
    int getMinimumCostId(std::map<unsigned int, RIBEntry *> Q);

  public:
    OSPFNode(int id, std::string remote_address, int remote_port, int local_port);
    ~OSPFNode();

    void service();
};

#endif
