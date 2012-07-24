/*
 * ospfnode.cc
 *
 * CS456 A2 OSPFNode Implementation
 * Written by Andrew J Klamut (ajklamut)
 *
 * July 2012
 *
 * This file has the OSPFNode class implementation.
 */

#include "ospfnode.h"
#include <cstdlib>
#include <cstring>
#include <netdb.h>
#include <arpa/inet.h>

using namespace std;


/******************************************
 *** Main Control Flow Member Functions ***
 *****************************************/

OSPFNode::OSPFNode(int id, string remote_address, int remote_port, int local_port) : debug("debug"+intToAscii(id)+".log"), output("router"+intToAscii(id)+".log") {
    debug << "[OSPFNode::OSPFNode] Starting with " <<
                                            " id " << id <<
                                            ", remote_address " << remote_address <<
                                            ", remote_port " << remote_port << 
                                            ", and local_port " << local_port << endl;
    this->id = id;
    this->remote_address = remote_address;
    this->remote_port = remote_port;
    this->local_port = local_port;
    this->remote_fd = -1;

    try {
        establishConnection();
    } catch (SocketError e) {
        debug << "[OSPFNode::OSPFNode] Caught error from OSPFNode::establishConnections: " << e.msg << endl;
        exit(EXIT_FAILURE);
    }

    sendInitPacket();
    getCircuitDatabase();
    printCircuitDatabase();

    // Add the ciruit database to the topography database and update the RIB each time
    for (map<unsigned int, unsigned int>::iterator it = circuit_db.begin(); it != circuit_db.end(); it++) {
        addTopologyEntry(id, it->first, it->second);
        rebuildRIB();
    }

    // Send hellos
    for (map<unsigned int, unsigned int>::iterator it = circuit_db.begin(); it != circuit_db.end(); it++) {
        debug << "[OSPFNode:OSPFNode] Sending HELLO with router_id " << id << " and link_id " << it->first << endl;
        sendHelloPacket(it->first);
    }

    debug << "[OSPFNode::OSPFNode] Finished." << endl << endl;
}

OSPFNode::~OSPFNode() {
    // Since we already call freeaddrinfo() in establishConnection(), closing the the socket is the only networking-related teardown we need to do
    close(remote_fd);

    // Delete any TopologyEntry objects allocated on the heap
    for (map<unsigned int, TopologyEntry *>::iterator it = topology_db.begin(); it != topology_db.end(); it++) {
        delete it->second;
    }

    // Delete any RIBEntry objects allocated on the heap
    for (map<unsigned int, RIBEntry *>::iterator it = rib.begin(); it != rib.end(); it++) {
        delete it->second;
    }
}

void OSPFNode::service() {
    debug << "[OSPFNode::service] Starting." << endl;

    // At this point we are only interested in receiving pkt_HELLO or pkt_LSPDU, the latter of which is bigger
    unsigned int max_packet_size = sizeof(int)*5, buffer_index = 0, nbytes;
    char *buffer = new char[max_packet_size], *int_buffer = new char[sizeof(int)];

    while (true) {
        // Receive the next packet
        nbytes = recvMessage(buffer, max_packet_size);
        buffer_index = 0;

        if (nbytes == sizeof(int)*2) {
            // pkt_HELLO has 2 integer members
            unsigned int router_id, link_id;

            // Parse the 2 received members
            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            router_id = charsToInt(int_buffer);

            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            link_id = charsToInt(int_buffer);

            debug << "[OSPFNode::service] Got pkt_HELLO with router_id " << router_id << " and link_id " << link_id << endl;
            output << "R" << id << " receives a HELLO: router_id " << router_id << ", link_id " << link_id << endl;

            // Respond to the router that sent this pkt_HELLO with pkt_LSPDUs including *only* our circuit db info
            for (map<unsigned int, unsigned int>::iterator it = circuit_db.begin(); it != circuit_db.end(); it++) {
                sendLSPDUPacket(id, id, it->first, it->second, link_id);
            }

            // Add this link to the list of active links, to be used for forwarding received LSPDU packets
            active_links.push_back(link_id);
        } else {
            // pkt_LSPDU has 5 integer members
            unsigned int sender, router_id, link_id, cost, via;
            int neighbour_id;

            // Parse each received member
            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            sender = charsToInt(int_buffer);

            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            router_id = charsToInt(int_buffer);

            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            link_id = charsToInt(int_buffer);

            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            cost = charsToInt(int_buffer);

            for (int i = 0; i < sizeof(int); i++) {
                int_buffer[i] = buffer[buffer_index++];
            }
            via = charsToInt(int_buffer);
            debug << "[OSPFNode::service] Got pkt_LSPDU with sender " << sender << ", router_id " << router_id << ", link_id " << link_id << ", cost " << cost << ", and via " << via << endl;
            output << "R" << id << " receives an LSPDU: sender " << sender << ", router_id " << router_id << ", link_id " << link_id << ", cost " << cost << ", via " << via << endl;

            // If this is not new information, skip
            if (topology_db.count(router_id) > 0 && topology_db[router_id]->link_costs.count(link_id) > 0) {
                debug << "[OSPFNode::service] Skipping redundant information." << endl;
                continue;
            }

            addTopologyEntry(router_id, link_id, cost);
            rebuildRIB();
            forwardLSPDUPacket(buffer, via);
        }
    }

    // Cleanup
    delete[] buffer, int_buffer;
    debug << "[OSPFNode::service] Finishing." << endl << endl;
}


/***************************************
 *** Socket-Related Member Functions ***
 **************************************/

void OSPFNode::establishConnection() throw (SocketError) {
    debug << "[OSPFNode::establishConnection] Starting." << endl;
    debug << "[OSPFNode::establishConnection] About to create socket on local port " << local_port << endl;

    // Setup socket structs as per Beej's tutorial and get local address info
    struct addrinfo hints, *servinfo, *p;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE;

    if (getaddrinfo(NULL, intToAscii(local_port).c_str(), &hints, &servinfo) != 0) {
        debug << "[OSPFNode::establishConnection] Error during socket creation getting local address info." << endl;
        throw SocketError("Error getting local address info.");
    }

    // Create a remote socket and bind it to the local port. We will use the same socket for sending and receiving
    for (p = servinfo; p != NULL; p = p->ai_next) {
        remote_fd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (remote_fd == -1) {
            continue;
        }
        if (bind(remote_fd, p->ai_addr, p->ai_addrlen) == -1) {
            close(remote_fd);
            continue;
        }
        break;
    }

    // If here we never hit the break in the previous loop; die
    if (p == NULL) {
        debug << "[OSPFNode::establishConnection] Couldn't create a socket." << endl;
        throw SocketError("Error creating a socket.");
    }
    freeaddrinfo(servinfo); // Sort of a delete on servinfo

    debug << "[OSPFNode::establishConnection] Socket successfully bound to local port " << local_port << "!" << endl;
    debug << "[OSPFNode::establishConnection] About to try connecting the socket to remote port " << remote_port << "..." << endl;

    // Do the get address info thing again with the structs declared above, but for the remote address
    if (getaddrinfo(remote_address.c_str(), intToAscii(remote_port).c_str(), &hints, &servinfo) != 0) {
        debug << "[OSPFNode::establishConnection] Error getting remote address info." << endl;
        throw SocketError("Error getting remote address info.");
    }

    // Connect so that we can use send() and receive() later, without specifying extra stuff
    if (connect(remote_fd, servinfo->ai_addr, servinfo->ai_addrlen) == -1) {
        debug << "[OSPFNode::establishConnection] Error connecting socket to remote address." << endl;
        throw SocketError("Error creating connection.");
    }
    freeaddrinfo(servinfo); // Sort of a delete on servinfo, again

    debug << "[OSPFNode::establishConnection] Socket successfully connected to remote port " << remote_port << "!" << endl;
    debug << "[OSPFNode::establishConnection] Finished." << endl << endl;
}

int OSPFNode::sendMessage(char *buffer, unsigned int length) throw (SocketError) {
    // Temporarily copy this over so it can be printed, by adding the '\0'
    char temp_buffer[length+1];
    for (int i = 0; i < length; i++) {
        temp_buffer[i] = buffer[i];
    }
    temp_buffer[length] = '\0';
    debug << "[OSPFNode::sendMessage] Starting with buffer \"" << temp_buffer << "\"" << endl;

    int nbytes = -1;
    nbytes = send(remote_fd, buffer, length, 0);

    // Check if the send failed, or if it wasn't able to send the amount specified by the length argument
    if (nbytes == -1) {
        debug << "[OSPFNode::sendMessage] Error sending message." << endl;
        throw SocketError("Error sending message.");
    } else if (nbytes != length) {
        debug << "[OSPFNode::sendMessage] Couldn't send the requested " << length << " bytes. Instead I sent " << nbytes << " bytes." << endl;
        throw SocketError("Error sending part of message.");
    }

    debug << "[OSPFNode::sendMessage] Ending with nbytes " << nbytes << endl << endl;
    return nbytes;
}

int OSPFNode::recvMessage(char *buffer, unsigned int length) throw (SocketError) {
    debug << "[OSPFNode::recvMessage] Starting." << endl;

    int nbytes = -1;
    nbytes = recv(remote_fd, buffer, length, 0);

    // Check if the receive failed, or if it wasn't able to receive the amount specified by length
    if (nbytes == -1) {
        debug << "[OSPFNode::recvMessage] Error receiving message." << endl;
        throw SocketError("Error receiving message.");
    }

    debug << "[OSPFNode::recvMessage] Ending with nbytes " << nbytes << endl << endl;
    return nbytes;
}


/****************************************
 *** Packet-Specific Member Functions ***
 ***************************************/

void OSPFNode::sendInitPacket() {
    debug << "[OSPFNode::sendInitPacket] About to send INIT packet with data " << id << endl;

    // Create a character buffer, marshall our int into it, and send it to the nse
    char *init_packet = new char[sizeof(int)];
    init_packet = intToChars(id, init_packet);
    sendMessage(init_packet, sizeof(int));

    delete[] init_packet;
    init_packet = NULL;

    output << "R" << id << " sends an INIT: router_id " << id << endl;
    debug << "[OSPFNode::sendInitPacket] INIT packet sent." << endl << endl;
}

void OSPFNode::getCircuitDatabase() {
    debug << "[OSPFNode::getCircuitDatabase] About to wait on receiving circuit DB..." << endl;

    // Max packet size is NBR_ROUTER of link_cost structs (which is really just 2 ints) plus the nbr_link member
    unsigned int max_packet_size = NBR_ROUTER*(sizeof(int)*2) + sizeof(int);
    debug << "[OSPFNode::getCircuitDatabase] Going to receive " << max_packet_size << " bytes." << endl;

    char *circuit_packet = new char[max_packet_size], *int_buffer = new char[sizeof(int)];
    recvMessage(circuit_packet, max_packet_size);
    unsigned int packet_index = 0;

    // Copy the nbr_link member from the master packet buffer
    for (int i = 0; i < sizeof(int); i++) {
        int_buffer[i] = circuit_packet[packet_index++];
    }
    int nbr_link = charsToInt(int_buffer);
    debug << "[OSPFNode::getCircuitDatabase] Got nbr_link value " << nbr_link << " and packet_index is at " << packet_index << endl;

    // Start the link state DB with only information for this router 
    unsigned int link, cost;

    output << "R" << id << " receives a circuit_DB: nbr_link " << nbr_link;
    debug << "[OSPFNode::getCircuitDatabase] About to parse this router (" << id << "'s) circuit database:" << endl;

    // Get <nbr_link> pairs of ints
    for (int i = 0; i < nbr_link; i++) {
        // Get the first member of the current link cost struct
        for (int j = 0; j < sizeof(int); j++) {
            int_buffer[j] = circuit_packet[packet_index++];
        }
        link = charsToInt(int_buffer);
        debug << "[OSPFNode::getCircuitDatabase] Got link id " << link << " and packet_index is at " << packet_index << endl;

        // Get the second member of the current link cost struct
        for (int j = 0; j < sizeof(int); j++) {
            int_buffer[j] = circuit_packet[packet_index++];
        }
        cost = charsToInt(int_buffer);
        debug << "[OSPFNode::getCircuitDatabase] Got link cost " << cost << " and packet_index is at " << packet_index << endl;

        circuit_db.insert(make_pair(link, cost));
        output << ", linkcost[" << i << "]: link " << link << ", cost " << cost;
    }

    // Cleanup
    delete[] circuit_packet;
    circuit_packet = NULL;

    output << endl;
    debug << "[OSPFNode::getCircuitDatabase] Finished and packet_index is at " << packet_index << endl << endl;
}

void OSPFNode::printCircuitDatabase() {
    debug << "[OSPFNode::printLinkStateDB] This router's circuit database is:" << endl;
    for (map<unsigned int, unsigned int>::iterator it = circuit_db.begin(); it != circuit_db.end(); it++) {
        debug << "[OSPFNode::printLinkStateDB] Connected to L" << it->first << " with cost " << it->second << endl;
    }
    debug << endl;
}

void OSPFNode::sendHelloPacket(unsigned int link_id) {
    debug << "[OSPFNode::sendHelloPacket] Starting with link_id " << link_id << endl;
    char *main_buffer = new char[sizeof(int)*2], *int_buffer = new char[sizeof(int)];
    unsigned int main_index = 0;

    // Get the router_id part first
    int_buffer = intToChars(id, int_buffer);

    for (int i = 0; i < sizeof(int); i++) {
        main_buffer[main_index++] = int_buffer[i];
    }

    // Get the link_id part next
    int_buffer = intToChars(link_id, int_buffer);

    for (int i = 0; i < sizeof(int); i++) {
        main_buffer[main_index++] = int_buffer[i];
    }

    sendMessage(main_buffer, sizeof(int)*2);

    delete[] int_buffer, main_buffer;
    output << "R" << id << " sends a HELLO: router_id " << id << ", link_id " << link_id << endl;
    debug << "[OSPFNode::sendHelloPacket] Finishing." << endl << endl;
}

void OSPFNode::sendLSPDUPacket(unsigned int sender, unsigned int router_id, unsigned int link_id, unsigned int cost, unsigned int via) {
    debug << "[OSPFNode::sendLSPDUPacket] Starting with sender " << sender << ", router_id " << router_id << ", link_id " << link_id << ", cost " << cost << ", and via " << via << endl;

    char *buffer = new char[sizeof(int)*5], *int_buffer = new char[sizeof(int)];
    unsigned int buffer_index = 0, i;

    // Convert sender to characters and add to main buffer
    int_buffer = intToChars(sender, int_buffer);
    for (i = 0; i < sizeof(int); i++) {
        buffer[buffer_index++] = int_buffer[i];
    }

    // Convert router_id to characters and add to main buffer
    int_buffer = intToChars(router_id, int_buffer);
    for (i = 0; i < sizeof(int); i++) {
        buffer[buffer_index++] = int_buffer[i];
    }

    // Convert link_id to characters and add to main buffer
    int_buffer = intToChars(link_id, int_buffer);
    for (i = 0; i < sizeof(int); i++) {
        buffer[buffer_index++] = int_buffer[i];
    }

    // Convert cost to characters and add to main buffer
    int_buffer = intToChars(cost, int_buffer);
    for (i = 0; i < sizeof(int); i++) {
        buffer[buffer_index++] = int_buffer[i];
    }

    // Convert via to characters and add to main buffer
    int_buffer = intToChars(via, int_buffer);
    for (i = 0; i < sizeof(int); i++) {
        buffer[buffer_index++] = int_buffer[i];
    }

    sendMessage(buffer, sizeof(int)*5);
    output << "R" << id << " sends an LSPDU: sender " << sender << ", router_id " << router_id << ", link_id " << link_id << ", cost " << cost << ", via " << via << endl;
}

void OSPFNode::forwardLSPDUPacket(char *buffer, unsigned int via) {
    debug << "[OSPFNode::forwardLSPDUPacket] Starting." << endl;

    char *int_buffer = new char[sizeof(int)];
    int_buffer = intToChars(id, int_buffer);

    // Change the sender field of the buffer to this router's id (first size(int) bytes)
    for (int i = 0; i < sizeof(int); i++) {
        buffer[i] = int_buffer[i];
    }

    // Send this slightly modified packet to everyone on the active link list except the original sender
    for (vector<unsigned int>::iterator it = active_links.begin(); it != active_links.end(); it++) {
        if (*it == via) {
            continue;
        }

        // Get the value of the next link for filling in the via field
        int_buffer = intToChars(*it, int_buffer);

        for (int i = 0; i < sizeof(int); i++) {
            buffer[sizeof(int)*4+i] = int_buffer[i];
        }
        sendMessage(buffer, sizeof(int)*5);
        output << "R" << id << " forwards the last received LSPDU: via link " << *it << endl;
    }

    delete[] int_buffer;
    debug << "[OSPFNode::forwardLSPDUPacket] Finishing." << endl;
}


/******************************************
 *** Database-Specific Member Functions ***
 *****************************************/

void OSPFNode::addTopologyEntry(unsigned int router_id, unsigned int link_id, unsigned int cost) {
    debug << "[OSPFNode::addTopologyEntry] Starting with router_id " << router_id << ", link_id " << link_id << ", and cost " << cost << endl;
    TopologyEntry *t = NULL;

    // First check if we have a spot for this particular router, and create one if not
    if (topology_db.count(router_id) == 0) {
        debug << "[OSPFNode::addTopologyEntry] Creating entry for the first time for router " << router_id << endl;
        t = new TopologyEntry();
        topology_db.insert(make_pair(router_id, t));
    }

    // Now add the actual link information to the router's list
    t = topology_db[router_id];
    t->link_costs.insert(make_pair(link_id, cost)); // This will not affect the t->link_costs map if the key is redundant (built-in to std::map::insert)

    printTopologyDatabase();

    debug << "[OSPFNode::addTopologyEntry] Finished." << endl << endl;
}

void OSPFNode::printTopologyDatabase() {
    // Print the output of the topology database, as specified in the assignment
    output << endl << "# Topology database" << endl;
    for (map<unsigned int, TopologyEntry *>::iterator t_it = topology_db.begin(); t_it != topology_db.end(); t_it++) {
        output << "R" << id << " -> R" << t_it->first << " nbr link " << t_it->second->link_costs.size() << endl;
        for (map<unsigned int, unsigned int>::iterator l_it = t_it->second->link_costs.begin(); l_it != t_it->second->link_costs.end(); l_it++) {
            output << "R" << id << " -> R" << t_it->first << " link " << l_it->first << " cost " << l_it->second << endl;
        }
    }
}

void OSPFNode::rebuildRIB() {
    debug << "[OSPFNode::rebuildRIB] Starting." << endl;
    RIBEntry *r = NULL;
    int neighbour_id = -1;

    // This loop adds anything from the topology database (which may be new) to the RIB
    for (map<unsigned int, TopologyEntry *>::iterator t_it = topology_db.begin(); t_it != topology_db.end(); t_it++) {
        // First check if we have a spot for this particular router, and create one if not
        if (rib.count(t_it->first) == 0) {
            debug << "[OSPFNode::rebuildRIB] Creating entry for the first time for router " << t_it->first << endl;
            r = new RIBEntry();
            rib.insert(make_pair(t_it->first, r));

            if (t_it->first == id) {
                // Special case: We are adding an entry for ourselves. Say cost is 0 and that we want to hop to self
                r->next_hop = id;
                r->min_cost = 0;
            } else {
                // Else initialize to default values: Hop to self and very large cost
                r->next_hop = t_it->first;
                r->min_cost = 0xFFFFFFFF; // Fill in all the bits for a maximum value. Don't worry, it's an unsigned int
            }
        }

        // Now add the actual link information to the router's list
        r = rib[t_it->first];

        for (map<unsigned int, unsigned int>::iterator l_it = t_it->second->link_costs.begin(); l_it != t_it->second->link_costs.end(); l_it++) {
            neighbour_id = getNeighbourId(t_it->first, l_it->first);

            if (neighbour_id != -1) {
                debug << "[OSPFNode::rebuildRIB] Adding to entry " << t_it->first << " neighbour " << neighbour_id << " with cost " << l_it->second << endl;
                r->neighbour_costs.insert(make_pair(neighbour_id, l_it->second));
            }
        }
    }

    // This part performs Dijkstra's on the RIB
    RIBEntry *origin = rib[id];
    unsigned int next_node, new_cost;
    map<unsigned int, RIBEntry *> Q = rib;
    debug << "[OSPFNode::rebuildRIB] About to perform Dijkstra's." << endl;

    while (!Q.empty()) {
        next_node = getMinimumCostId(Q); // As per the algorithm, get the node with the least cost
        if (next_node == -1) break; // If none of the nodes have a reasonable cost (incomplete RIB), stop.
        r = Q[next_node];
        debug << "[OSPFNode::rebuildRIB] Got next_node " << next_node << " with current cost " << r->min_cost << endl;
        Q.erase(next_node); // "Add this to the cloud set"

        // Loop through all of the current node's neighbours, trying to find lower costs
        for (map<unsigned int, unsigned int>::iterator it = r->neighbour_costs.begin(); it != r->neighbour_costs.end(); it++) {
            new_cost = r->min_cost + it->second;
            debug << "[OSPFNode::rebuildRIB] For next_node " << next_node << ", got new cost " << new_cost << endl;

            // Found a better cost for a particular neighbour
            if (Q.count(it->first) > 0 && new_cost < Q[it->first]->min_cost) {
                Q[it->first]->min_cost = new_cost;
                debug << "[OSPFNode::rebuildRIB] Set node " << it->first << "'s cost to " << Q[it->first]->min_cost << endl;

                // Set the hop based on either being a neighbour to the origin node, or recursively using the current node's next_hop
                if (origin->neighbour_costs.count(it->first) > 0) {
                    Q[it->first]->next_hop = it->first;
                } else {
                    Q[it->first]->next_hop = r->next_hop;
                }

                debug << "[OSPFNode::rebuildRIB] Setting node " << it->first << "'s next_hop to " << Q[it->first]->next_hop << endl;
            }
        }
        debug << "[OSPFNode::rebuildRIB] -----------Done Dijkstra iteration-----------" << endl;
    }

    printRIB();

    debug << "[OSPFNode::rebuildRIB] Finished." << endl << endl;
}

void OSPFNode::printRIB() {
    // Print the output of the RIB, as specified in the assignment
    output << endl << "# RIB" << endl;
    for (map<unsigned int, RIBEntry *>::iterator it = rib.begin(); it != rib.end(); it++) {
        if (it->first == id) {
            output << "R" << id << " -> R" << it->first << " -> Local, 0" << endl;
        } else {
            output << "R" << id << " -> R" << it->first << " -> R" << it->second->next_hop << ", " << it->second->min_cost << endl;
        }
    }

    output << endl;
}

int OSPFNode::getNeighbourId(unsigned int router_id, unsigned int link_id) {
    debug << "[OSPFNode::getNeighbourId] Finding the router who neighbours " << router_id << " on link " << link_id << endl;
    for (map<unsigned int, TopologyEntry *>::iterator t_it = topology_db.begin(); t_it != topology_db.end(); t_it++) {
        // Skip the entry for the source router we already know about
        if (t_it->first == router_id) {
            continue;
        }

        for (map<unsigned int, unsigned int>::iterator l_it = t_it->second->link_costs.begin(); l_it != t_it->second->link_costs.end(); l_it++) {
            // Check if the current router, that is not the same as the provided router_id, is on the other end of this link
            if (l_it->first == link_id) {
                debug << "[OSPFNode::getNeighbourId] Found neighbour " << t_it->first << endl;
                return t_it->first;
            }
        }
    }
    debug << "[OSPFNode::getNeighbourId] Couldn't find a neighbour, returning -1." << endl;
    return -1;
}

int OSPFNode::getMinimumCostId(map<unsigned int, RIBEntry *> Q) {
    int min_id = -1;
    unsigned int min_cost = 0xFFFFFFFF;

    for (map<unsigned int, RIBEntry *>::iterator it = Q.begin(); it != Q.end(); it++) {
        if (Q.count(it->first) > 0 && it->second->min_cost < min_cost) {
            min_cost = it->second->min_cost;
            min_id = it->first;
        }
    }

    return min_id;
}
