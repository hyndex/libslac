# HomePlug Green PHY: The Standard For In-Home Smart Grid Powerline Communications

*Copyright © 2010, HomePlug Powerline Alliance, Inc. All rights reserved.*
*Version 1.00, 14-June-2010*

## Table of Contents

* **Executive Summary** – page 3
* **Driving Forces and Trends in The Smart Grid** – page 4

  * Home Area Network vs. Wide Area Network – page 4
  * Reliable Communications for Diverse Consumer Devices – page 5
  * Compatibility with other In-Home Networking Applications – page 5
  * Ubiquitous Networking is Critical for the Smart Grid – page 5
  * Low-cost and Low-power are Fundamental for “Green” Networking – page 5
  * Both Wired and Wireless Approaches Are Needed – page 5
  * Summary of Requirements and Key Initiatives by NIST and IEEE – page 5
* **HomePlug’s Role in the Smart Grid Evolution** – page 6

  * Early and Obvious Choice for Powerline Networking – page 6
  * Active Collaborator with Utilities on Wired Alternatives – page 6
  * Innovator to Leverage Low-cost, Low-power Version of Proven Standards – page 6
  * Liaison with ZigBee to Provide Wired/Wireless Compatibility – page 7
  * Coordination with NIST and IEEE to Drive Global Standards – page 7
* **The HomePlug Green PHY Standard** – page 7

  * Overview of HomePlug AV – page 8
  * Adaptive Bit Loading & Tone Maps – page 8
  * HomePlug AV MAC & Central Coordinator – page 9
  * CSMA and Channel Access Priority – page 10
  * HomePlug Green PHY – page 11
  * HomePlug GP Physical Layer – page 11
  * QPSK vs. 1024 QAM – page 11
  * ROBO Modes – page 12
  * HomePlug GP MAC – page 13
  * Distributed Bandwidth Control – page 13
  * Power Save Mode for Green PHY – page 14
  * Plug-in Vehicle (PEV) Association – page 15
* **Conclusions** – page 17

## Executive Summary

Driven by a global push for improved energy management, the development of Smart Grid technology is also being fueled by a powerful combination of government mandates, utility industry initiatives, and consumer demand for lower energy costs. Developed and developing countries around the globe are exploring various initiatives to improve energy usage within electrical grids. Private and public utility companies are taking the lead in the active development and deployment of Smart Grid technologies, both as a way to optimize overall energy usage and also to meet the demands of their customers for lower costs and better energy management tools for use by consumers. In addition, the rise of significant new energy requirements – such as the need to charge electric vehicles – is driving the need for specialized capabilities within a comprehensive Smart Grid energy management strategy.

The success of Smart Grid deployments will fundamentally depend on the robustness, reliability and interoperability of the underlying technologies used for Home Area Network (HAN) Smart Grid communications. In addition to delivering low-cost, low-power and a ubiquitous reach to connect all electricity-using devices within the Smart Grid, these technologies will need to conform with industry-wide standards, meet strict certification criteria and also provide applications-level compatibility for both wired and wireless implementations.

HomePlug Green PHY has now emerged as the leading standard for implementing Smart Grid functionality across in-home powerline-based networks.

Based upon a series of on-going discussions that began years ago with leaders in the utility industry, the HomePlug Green PHY (HomePlug GP) Specification is a result of an intense standards development effort by the Smart Energy Technical Working Group within the HomePlug Powerline Alliance. Developed as a low-cost, low-power adaptation of the proven HomePlug AV standard, HomePlug GP also is fully compliant with the IEEE P1901 Draft Standard for Powerline Networks. In addition, HomePlug GP will benefit from the huge ecosystem of existing HomePlug AV technology solutions and the robust third-party certification systems already in place for assuring universal interoperability between HomePlug-enabled and IEEE P1901 devices.

As the global leader in establishing standards for powerline communications since the inception of the technology and the certification authority for IEEE P1901 devices, the HomePlug Powerline Alliance is uniquely positioned to drive standards for powerline-based Smart Grid solutions. In addition, the HomePlug Powerline Alliance and the ZigBee Alliance have already taken the initiative to create a joint Smart Energy standard (announced in 2009) that assures applications-level compatibility across both wired and wireless Smart Grid applications.

This whitepaper provides an overview of the driving forces, trends and industry requirements in the Smart Grid segment and reviews the HomePlug Powerline Alliance’s key role in the development and standardization of powerline-based Smart Grid technologies. The final section provides a detailed discussion of the HomePlug Green PHY specification, including the advantages that have accrued from basing HomePlug GP upon the proven HomePlug AV protocol, as well as the leverage from the huge ecosystem of HomePlug solutions that shorten time-to-market and the more than 45 million installed devices using HomePlug powerline technology.

## Driving Forces and Trends in The Smart Grid

In the broad sense, a Smart Grid is defined as *“delivering electricity from suppliers to consumers using two-way digital communications to control appliances at consumers’ homes, save energy, reduce cost and to increase reliability and transparency.”*

### Home Area Network vs. Wide Area Network

Obviously, a reliable communications system with a ubiquitous reach represents a critical element for successful implementation of the Smart Grid. Because of the different issues and requirements involved within the home vs. across the utility network, it is useful to separate these communications challenges into two segments. Generally, the utility side (from the electricity meter outward) can be referred to as the backhaul WAN, and the consumer side (within the home) is referred to as the Home Area Network (HAN).

![Diagram of Smart Grid HAN and WAN Communication Model](image-page4-diagram.png)

Experience with Smart Grid deployments to-date has already demonstrated the powerful potential for consumers to actively participate in energy conservation and to significantly reduce their overall electricity usage. Enhanced monitoring and control not only gives users the tools to improve their own energy efficiency, it drives participation in programs such as time-of-day pricing that lowers costs for users while aiding load-balancing for utilities. It also lays the groundwork for consumers to participate in peak-demand shut-down programs that provide pricing incentives to users who allow the Smart Grid to automatically curtail electricity usage by non-safety-critical in-home devices during peak demand situations. As other new electricity demands arise, such as the need to recharge electric vehicles during non-peak periods, the two-way communication capabilities of the Smart Grid will also make it possible to efficiently integrate these new requirements while maintaining optimal load-balancing throughout the overall utility grid.

### Reliable Communications for Diverse Consumer Devices

One major difference between the WAN and the HAN is the degree of control that the utility can exercise over the networking and communications environment. Utilities essentially can decide for themselves how best to implement communications across the WAN. However, the HAN is an entirely different situation, in which a wide range of devices selected by consumers need to be able to communicate seamlessly on the Smart Grid. Therefore, it is critical that HAN-based Smart Grid communications be based upon widely accepted and well-controlled standards, with enforceable certification mechanisms to assure reliability and interoperability of all connected devices.

### Compatibility with other In-Home Networking Applications

In addition, the Smart Grid communications within the HAN also must be able to co-exist compatibly and seamlessly with other home networking functions (such as audio-visual entertainment content, IPTV, interactive gaming, etc.), without causing any degradation or interference to these consumer-oriented applications. As will be discussed later, this can be a critical factor when implementing both Smart Grid and other home-based network applications using powerline technologies.

### Ubiquitous Networking is Critical for the Smart Grid

In order for the Smart Grid to deliver on the promise of comprehensive in-home energy management, it goes without saying that the HAN needs to include all of the devices using electricity throughout the residence. Basically, wherever any electric appliance, entertainment system, HVAC system or other device might be plugged in, there also needs to be a seamless connection to the Smart Grid.

In order to achieve full coverage throughout all types and ages of residences (old, new, single-family, condos, etc.), it’s also important that the selected Smart Grid communications technologies **not** impose the installation of a new wiring plant within the home, such as CAT5 or coax. Smart Grid communications need to provide a ubiquitous reach throughout the home by leveraging the existing wiring plant with powerline networking.

### Low-cost and Low-power are Fundamental for “Green” Networking

Another critical factor for achieving ubiquity in Smart Grid networking will be solutions that can be efficiently implemented in virtually every type of electricity-using device. This requires a complete network interface (PHY and MAC) that is optimized for both low-cost and low-power, along with clear standards-based documentation to support embedding of the interface within any device.

### Both Wired and Wireless Approaches Are Needed

From the beginning of Smart Grid development, utilities and their equipment suppliers have recognized the need for using both wireless and wired networking approaches. While wireless solutions such as ZigBee play a role in the Smart Grid, the signal propagation and physical barrier issues that are inherent with any wireless approach make it imperative to also have ubiquitous wired Smart Grid alternatives such as powerline. As discussed later, the ZigBee/HomePlug Smart Energy Profile addresses this issue by providing a common application layer enabling interoperability between the leading standards-based technologies for both wireless and wired Smart Grid applications.

### Summary of Requirements and Key Initiatives by NIST and IEEE

In summary, the key requirements that are critical for success of any Smart Grid communications technology include:

* Reliability across a diverse range of devices
* Compatibility and interoperability with existing consumer-driven home networking technologies
* Ubiquitous reach throughout the home (e.g. if you plug it in, it’s automatically on the Smart Grid network)
* Low-cost and low-power interfaces that can be “embedded” in any device
* Both wired and wireless solutions with application-level interoperability

As mandated by the Energy Independence and Security Act (EISA) of 2007, the National Institute of Standards and Technology (NIST) has **“primary responsibility to coordinate development of a framework that includes protocols and model standards for information management to achieve interoperability of smart grid devices and systems.”** In this role, NIST is actively collaborating with businesses, utilities and standards organizations to foster an ecosystem for Smart Grid implementation to support national efforts toward energy independence, conservation and global interoperability.

As the recognized global standards body for data communications, the Institute of Electrical and Electronics Engineers (IEEE) is also playing a key role through the development of the IEEE P1901 standard for Broadband Over Powerline Networks, which defines the Medium Access Control (MAC) and Physical Layer (PHY) specifications to implement powerline-based networks that can provide the foundation for Smart Grid communications.

## HomePlug’s Role in the Smart Grid Evolution

### Early and Obvious Choice for Powerline Networking

Over three years ago, as Smart Grid technologies were initially being developed and tested, the utility industry naturally looked toward existing powerline networking technologies as a potential communications methodology. As the recognized leader in powerline networking standards for consumer and home control applications, the HomePlug Powerline Alliance immediately moved into a key role of collaborator with these early development efforts.

### Active Collaborator with Utilities on Wired Alternatives

Two years ago, at ConnectivityWeek 2008, this collaborative relationship was stepped up to a higher level as a number of key representatives from the utility industry met with the HomePlug Powerline Alliance and more formally defined their requirements. The consensus among the utilities was a definite need for a wired powerline alternative because existing wireless technologies were not sufficient for Smart Grid implementation in all cases. Furthermore, the utilities provided a consensus desire for a lower cost and lower power version of the widely accepted HomePlug AV powerline networking specification, along with the view that a 1 Mbps data rate would be sufficient to meet their Smart Grid requirements.

### Innovator to Leverage Low-cost, Low-power Version of Proven Standards

In response to these inputs and in concert with its on-going standards leadership, the HomePlug Powerline Alliance established a dedicated effort to develop the HomePlug Green PHY (HomePlug GP) Specification for powerline communications that are optimized to meet utility industry requirements for Smart Grid applications.

By leveraging the Alliance’s broad base of industry players, the Smart Energy Technical Working Group was formed including the following: Arkados, Atheros, Cisco, Duke, GE, Gigle, Marvell, PG\&E, Renesas Electronics, SPiDCOM, ST Microelectronics, and Watteco. Inputs regarding real-world utility requirements were provided by Consumers Energy, Duke Energy, Pacific Gas & Electric, Reliant Energy, Southern California Edison and others.

The resultant HomePlug GP specification leverages the Alliance’s proven and widely deployed HomePlug AV specification by creating a streamlined version with 25% of the cost and 25% of the power consumption, while still delivering ample bandwidth to meet the Smart Grid communication requirements. In addition, the HomePlug GP specification is interoperable with HomePlug AV, thereby assuring utilities of shared-network compatibility with the 45 million HomePlug enabled devices that have already shipped (representing over 80 percent of the world’s broadband powerline market) as well as with the continuing growth of HomePlug as the leading powerline interface for consumer devices. *(Details on the completed HomePlug Green PHY Specification are provided in the final section of this whitepaper.)*

### Liaison with ZigBee to Provide Wired/Wireless Compatibility

The HomePlug Powerline Alliance also undertook a joint standards effort with the ZigBee Alliance to create a specification for applications layer compatibility between the leading wired and wireless Smart Grid standards. The resultant Smart Energy Profile, announced in June 2009, is supported by a comprehensive certification process that delivers secure, robust, reliable, plug-and-play interoperability within the HAN for Advanced Metering Infrastructure (AMI) and Smart Grid applications.

### Coordination with NIST and IEEE to Drive Global Standards

As an active participant in the IEEE standards process and with the NIST Smart Grid initiative, the HomePlug Powerline Alliance has also worked to help create a globally accepted set of objectives and standards for Smart Grid networking. For example, Atheros, a member of the HomePlug GP Technical Working Group, received a U.S. Department of Energy grant to accelerate development of HomePlug GP standard-based ICs (one of only 8 grants that were awarded to a non-utility out of over 400 grant submissions). Also, with its already well-established third-party certification program, the HomePlug Powerline Alliance will act as the certification body for IEEE P1901 devices to assure full compliance and interoperability.

In effect, the HomePlug Powerline Alliance is uniquely positioned to provide industry-wide coordination, communication and certification services to assure that efforts by NIST, IEEE and the utilities industry achieve the targeted Smart Grid objectives of reliability, compatibility, ubiquity and performance.

## The HomePlug Green PHY Standard

As described in previous sections, the HomePlug GP specification was developed specifically to support Smart Grid applications on the Home Area Network (HAN) within the customer premises. The HomePlug Powerline Alliance had to find a means of reducing cost and power consumption while maintaining HomePlug AV/P1901 interoperability, reliability and coverage. It was essential to guarantee that HomePlug GP devices would not adversely affect broadband services such as IPTV and video distribution that are already being supported by HomePlug AV.

The HomePlug GP specification is the result of more than one year of dedicated work by the HomePlug Smart Energy Technical Working Group. Through their efforts, HomePlug GP meets all of the objectives and requirements set forth by the utility industry. It is a simple but powerful extension of HomePlug AV that offers ample bandwidth for both current and future Smart Grid applications.

### Overview of HomePlug AV

In order to lay the foundation for describing HomePlug GP, it’s necessary to briefly summarize the key attributes of the proven HomePlug AV technology from which it is derived. The HomePlug AV specification includes both MAC and PHY layers and is completely interoperable with the IEEE P1901 OFDM draft standard.

The following description of the HomePlug AV specification is necessarily abridged. Emphasis is placed on those aspects of the HomePlug AV specification that were modified by the HomePlug Smart Energy Technical Working Group to create the HomePlug GP specification.

**Key Attributes of HomePlug AV PHY**

| Attribute                               | Value                                                                |
| --------------------------------------- | -------------------------------------------------------------------- |
| Spectrum                                | 2 MHz to 30 MHz                                                      |
| Modulation                              | OFDM                                                                 |
| # Subcarriers                           | 1155                                                                 |
| Subcarrier spacing                      | 24.414 kHz                                                           |
| Supported subcarrier modulation formats | BPSK, QPSK, 16 QAM, 64 QAM, 256 QAM, 1024 QAM                        |
| Data FEC                                | Turbo code Rate 1/2 or Rate 16/21 (punctured)                        |
| Rate                                    | –                                                                    |
| Supported data rates                    | ROBO: 4 Mbps to 10 Mbps<br>Adaptive Bit Loading: 20 Mbps to 200 Mbps |

HomePlug AV has a signal bandwidth of 28 MHz and operates on existing power lines in a frequency range of 2 MHz to 30 MHz. The HomePlug AV PHY is based on Orthogonal Frequency Division Multiplexing (OFDM), which distributes information among many closely spaced narrowband subcarriers rather than on a single broadband carrier.

### Adaptive Bit Loading & Tone Maps

Any broadband signal transmitted on home power lines will encounter multipath distortion. As the name implies, multipath distortion occurs because transmitted signals can typically reach a receiver over more than one path. When these signals recombine at the receiver, different levels of attenuation are encountered at different frequencies within the signal passband.

OFDM was selected as the basic modulation scheme for HomePlug AV due to its ability to provide extremely robust performance in the presence of frequency-selective fading. OFDM subcarriers undergoing frequency-selective fading can arrive at the receiver at radically different signal levels.

HomePlug AV supports adaptive bit loading. Adaptive bit loading enables each subcarrier to be modulated in order to achieve the highest possible data rate based on the received signal strength. A subcarrier with low received signal strength might support Binary Phase Shift Keying (BPSK), which provides 2 bits per subcarrier per symbol. At the other extreme, a subcarrier with very high received signal strength could support 1024 QAM, which provides 10 bits per subcarrier per symbol.

Given that for high data rates (up to 200 Mbps), adaptive bit loading is unquestionably the best approach available on power lines. For lower-rate applications such as Smart Grid, however, considerable simplification is possible.

### HomePlug AV MAC & Central Coordinator

The HomePlug AV MAC uses Carrier Sense Multiple Access (CSMA) as the basic mandatory channel access scheme. Time Domain Multiple Access (TDMA) is supported as an optional feature.

Each AV Logical Network (AVLN) is controlled by one device in the network acting as a Central Coordinator (CCo). The CCo periodically transmits beacons to synchronize the network to the AC line cycle and to disseminate management messages. As shown in the following illustration, the beacon interval is two AC line cycles (33.3 msec @ 60 Hz or 40 msec @ 50 Hz).

The beacon is transmitted within a Beacon Region that immediately follows every second positive zero-crossing of the AC line cycle. Note that the beacon interval will accommodate multiple beacons. This is necessary because in some instances, multiple AVLNs can be within hearing range of each other. The Beacon Region allows multiple beacons to be transmitted without collision.

![Diagram of HomePlug AV beacon interval aligned with AC line cycle (beacon region and CSMA/TDMA regions shown)](image-page10-beacon-cycle.png)

Every HomePlug AV client can act as a CCo. If a CCo is removed from the network, any remaining client can assume the role of CCo. When this occurs, selection of the client best able to act as CCo is based on a set of pre-defined selection criteria.

### CSMA and Channel Access Priority

HomePlug AV uses Carrier Sense Multiple Access (CSMA) as the basic channel access mechanism. In addition, HomePlug AV supports four different Channel Access Priorities: CAP3 (highest priority) to CAP0 (lowest priority) to provide higher Quality of Service (QoS) for latency-sensitive applications such as video distribution.

CSMA is often described as “listen-before-talk.” Before transmitting a data packet, each device monitors the network. If a device is transmitting, other devices must defer transmission until the line becomes idle. When the medium is idle, network stations must resolve the highest priority of pending traffic before any individual station can access the medium for data transmission. This is accomplished during the Priority Resolution Period, which consists of two Priority Resolution Slots (PRS0 and PRS1).

![Diagram of HomePlug AV priority resolution period preceding medium contention (PRS0, PRS1, CIFS, etc.)](image-page10-priority-resolution.png)

During priority resolution, stations transmit Priority Resolution Symbols (PRS) based on the priority of queued traffic. All network clients must monitor the medium when not transmitting. A station contending for access at CAP3 or CAP2 would transmit during PRS0, causing stations contending at CAP1 or CAP0 to defer. A station contending at CAP3 would also transmit in PRS1, causing stations contending at CAP2 to defer. Once this process is complete, all stations are aware of the highest priority of pending traffic.

### HomePlug Green PHY

With the previous sections as background, it is important to re-emphasize that HomePlug Green PHY (HomePlug GP) is derived entirely from HomePlug AV. Therefore, HomePlug GP is completely interoperable with HomePlug AV and IEEE P1901. HomePlug GP and HomePlug AV devices can form either homogeneous or heterogeneous networks.

### HomePlug GP Physical Layer

HomePlug GP uses the same frequency band (2 MHz to 30 MHz), basic modulation scheme (OFDM), and Forward Error Correction (Turbo Codes) as HomePlug AV. The table below shows a comparison of HomePlug GP and HomePlug AV/P1901 PHY parameters. Note that the HomePlug GP PHY has some major simplifications.

**HomePlug GP PHY Simplifications Reduce Cost & Power Consumption**

| Parameter                                  | HomePlug AV                                                          | HomePlug GP                 |
| ------------------------------------------ | -------------------------------------------------------------------- | --------------------------- |
| Spectrum                                   | 2 MHz to 30 MHz                                                      | 2 MHz to 30 MHz             |
| Modulation                                 | OFDM                                                                 | OFDM                        |
| # Subcarriers                              | 1155                                                                 | 1155                        |
| Subcarrier spacing                         | 24.414 kHz                                                           | 24.414 kHz                  |
| Supported subcarrier modulation<br>formats | BPSK, QPSK, 16 QAM, 64 QAM,<br>256 QAM, 1024 QAM                     | QPSK only                   |
| Data FEC                                   | Turbo code<br>Rate 1/2 or Rate 16/21 (punctured)                     | Turbo code<br>Rate 1/2 only |
| PHY Supported data rates                   | ROBO: 4 Mbps to 10 Mbps<br>Adaptive Bit Loading: 20 Mbps to 200 Mbps | ROBO: 4 Mbps to 10 Mbps     |

One of the biggest differences between HomePlug AV and HomePlug GP is the peak PHY rate. HomePlug AV supports a peak PHY rate of 200 Mbps, which is simply not required for Smart Grid applications. Based on extensive discussion with the utility industry, it was learned that coverage and reliability were paramount considerations. Peak data rate could be reduced, which in turn would allow for reductions in both cost and power consumption.

HomePlug GP supports a peak PHY rate of 10 Mbps, which is the result of two key simplifications:

1. Restriction of OFDM subcarrier modulation exclusively to QPSK
2. Restriction to data rates supported by ROBO modes, thereby eliminating the need for adaptive bit loading and management of tone maps.

These measures were essential to enable low-cost, low-power devices that interoperate with HomePlug AV/P1901 while maintaining the same robust coverage and reliability.

### QPSK vs. 1024 QAM

In order to achieve a 200 Mbps peak PHY rate, HomePlug AV exploits very high orders of Quadrature Amplitude Modulation (QAM). The OFDM signal consists of 1155 subcarriers, of which 917 are data-bearing within the 2 MHz to 30 MHz band. HomePlug AV supports modulation constellations from BPSK to 1024 QAM. In comparison, HomePlug GP uses QPSK exclusively (as shown in the following table).

**HomePlug GP Uses QPSK Only**

| Subcarrier Modulation | Bits per Symbol | HomePlug AV | HomePlug GP |
| --------------------- | --------------- | ----------- | ----------- |
| BPSK                  | 1               | ✔           |             |
| QPSK                  | 2               | ✔           | ✔           |
| 16 QAM                | 4               | ✔           |             |
| 64 QAM                | 6               | ✔           |             |
| 256 QAM               | 8               | ✔           |             |
| 1024 QAM              | 10              | ✔           |             |

*✔ indicates support for modulation format*

By restricting modulation to QPSK, HomePlug GP devices cannot match the peak PHY rate of HomePlug AV. However, Smart Grid applications do not require the same data rates as those required for video distribution. The peak PHY rate of 10 Mbps for HomePlug GP is easily able to support currently envisioned Smart Grid applications and still provides ample capacity for future-proofing. At 10 Mbps, HomePlug GP is approximately 1000× faster than competing PLC technologies such as PRIME or G3, which operate at lower frequencies (<500 kHz).

*As shown above, QPSK is a much simpler form of modulation than 1024 QAM.* By making exclusive use of QPSK for subcarrier modulation, the analog front end and line driver requirements (linearity and converter resolution) are more benign. As a result, HomePlug GP devices should be able to achieve a higher level of integration, possibly including single-chip architectures, which will help reduce cost and footprint.

### ROBO Modes

Robust OFDM (ROBO) mode is a form of repeat coding that is used to support low rate / high reliability data transmission in HomePlug equipment. As shown in the following table, ROBO mode is used to support data rates from 4 Mbps to 10 Mbps. These data rates are supported by both HomePlug AV and HomePlug GP, which facilitates interoperability.

**ROBO Mode Data Rate Depends on Degree of Repeat Coding**

| Mode            | PHY Rate | # Repeat Copies |
| --------------- | -------- | --------------- |
| Mini-ROBO       | 3.8 Mbps | 5               |
| Standard ROBO   | 4.9 Mbps | 4               |
| High Speed ROBO | 9.8 Mbps | 2               |

In ROBO mode, information is transmitted redundantly on multiple subcarriers. All supported ROBO mode data rates (4 Mbps, 5 Mbps, and 10 Mbps) use QPSK modulation on all subcarriers and rate 1/2 Turbo Code FEC. Data rate is determined by the degree of repeat coding employed.

Use of repeat coding obviously introduces a degree of inefficiency, but there is a significant benefit. Due to the use of repeat coding, a significant fraction of subcarriers in a ROBO signal can be lost (either to interference or frequency-selective fading) without causing a packet error. ROBO mode transmissions have proven to be reliable under the most severe line conditions.

Unlike Adaptive Bit Loading, ROBO modes do not rely on advanced knowledge of channel conditions prior to packet transmission. By restricting HomePlug GP devices exclusively to ROBO modes, there is no need to record or manage tone maps for each link in the network. This is a significant simplification that reduces memory size and code space.

### HomePlug GP MAC

The HomePlug GP MAC is basically a simplified version of the HomePlug AV/P1901 MAC. The HomePlug GP MAC shares the same CSMA and Priority Resolution mechanisms as HomePlug AV. It does not, however, support the optional TDMA mechanism. Due to the fact that the HomePlug GP PHY does not use adaptive bit loading, the HomePlug GP MAC does not need to record or manage tone maps.

**Comparison of HomePlug GP and HomePlug AV MAC Features**

| Function           | HomePlug AV                                                         | HomePlug GP                                       |
| ------------------ | ------------------------------------------------------------------- | ------------------------------------------------- |
| Channel Access     | CSMA/CA with optional TDMA                                          | CSMA/CA only                                      |
| CCo capable?       | Yes                                                                 | Yes                                               |
| Channel Estimation | Adaptive bit-loading per subcarrier<br>via pre-negotiated tone maps | ROBO eliminates need for pre-negotiated tone maps |
| Bandwidth Sharing  | N/A                                                                 | ROBO eliminates need for pre-negotiated tone maps |

There are some features that are unique to the HomePlug GP MAC. In order to ensure that HomePlug GP devices would not adversely affect network throughput of HomePlug AV devices, a unique bandwidth sharing algorithm was developed. A novel power saving mechanism was added. Finally, a method for characterizing signal level attenuation was included to facilitate association & binding of electric vehicles with charging equipment in public parking areas.

### Distributed Bandwidth Control

HomePlug GP devices will be restricted to a 10 Mbps peak PHY rate. Given that the required MAC throughput for Smart Grid applications is <250 kbps, this is more than adequate. However, from the perspective of a heterogeneous network comprised of both HomePlug AV and HomePlug GP devices capable of supporting both video distribution as well as Smart Grid applications, 10 Mbps is relatively slow. A 1500-byte Ethernet packet requires a much longer transmission time at 10 Mbps than it does at 200 Mbps.

If HomePlug GP devices operating in the presence of heavy HomePlug AV voice or video traffic were able to access the medium in an unconstrained manner, it is quite possible that HomePlug AV throughput could be adversely affected. Distributed Bandwidth Control (DBC) was included as a mandatory element of the HomePlug GP specification in order to ensure that HomePlug GP devices would not adversely impact existing HomePlug AV services.

When traffic at CAP3, CAP2, or CAP1 is detected, DBC will limit aggregate HomePlug GP channel access time, or “Time-on-Wire” (ToW), to approximately 7%. This corresponds to an effective PHY rate of 700 kbps (7% ToW @ 10 Mbps) and a MAC throughput rate of 400–500 kbps, which provides ample capacity for Smart Grid applications. HomePlug GP clients monitor all HomePlug GP transmissions in a 33.3 msec sliding window that immediately precedes any attempt to contend for channel access. This is possible because HomePlug GP packets have a special flag in the Start of Frame (SoF) delimiter.

If the pending packet will cause aggregate HomePlug GP ToW to exceed 7%, the HomePlug GP client cannot contend for channel access and must wait for an ensuing channel access opportunity. However, in most instances, the local medium will not be completely occupied. HomePlug GP devices may exploit unused ToW without restriction. This is accomplished by contending for channel access at CAP0. In the limit, HomePlug GP equipment installed in locations in which HomePlug AV equipment is not present may occupy up to 100% ToW.

### Power Save Mode for Green PHY

Reduced power consumption is a critical factor for Smart Grid applications. A special Power Save Mode has been developed to enable dramatic power savings for HomePlug GP devices. In order to describe the Power Save Mode, some terms must be explained:

1. **Awake Window:** Period of time during which the device is capable of transmitting and receiving frames on the power line. The Awake Window has a range of 1.5 msec to 2.1 seconds.
2. **Sleep Window:** Period of time during which the device is not capable of receiving or transmitting frames over the power line.
3. **Power Save Period (PSP):** PSP is the sum of the Awake and Sleep Windows. The PSP only has discrete values that are equivalent to 2^n beacon periods, up to a maximum value of 2^10 (1024) beacon periods.
4. **Power Save Schedule (PSS):** PSS conveys the PSP and the duration of the Awake Window. The Awake State always occurs at the beginning of the PSP.

It is essential that all devices operating in Power Save Mode enter and exit the Awake State in a coordinated manner. This ensures that the Awake Windows are overlapped to the maximum degree possible, thus enabling communication among devices operating in Power Save Mode. In the absence of such coordination, HomePlug GP devices would awake at different times and would be unable to communicate directly among each other.

![Diagram of HomePlug GP Power Save Mode schedule achieving maximum overlap of Awake windows among multiple stations](image-page15-power-save.png)

Stations must transmit a message to the CCo requesting permission to enter Power Save Mode. The request message includes the PSS. The CCo acknowledges the request to enter Power Save Mode and specifies the beacon that the requested PSP will commence on. Note that the PSP is different for each device. However, all PSPs are 2^n beacon periods in duration. Thus, longer PSPs are always an integer multiple of shorter PSPs. This allows the CCo to align the various PSPs for maximum overlap of the Awake Windows.

HomePlug GP Power Save Mode allows users to specify very long PSPs for aggressive power savings or shorter PSPs for lower latency and response times. Recall the PSPs have a duration from 1 beacon period (33.3 msec @ 60 Hz or 40 msec @ 50 Hz AC line cycle) up to 1024 beacon periods (34.1 sec @ 60 Hz or 41 sec @ 50 Hz). If a latency of 30–40 seconds is acceptable, HomePlug GP devices can reduce average power consumption by 97% relative to a device that is in the Awake State at all times. Devices with latencies of just 160 msec (2 beacon periods @ 50 Hz) could reduce power by more than 85%.

### Plug-in Vehicle (PEV) Association

Plug-in Electric Vehicle (PEV) charging is a major Smart Grid application. Long term, the goal is to provide the means of charging PEVs at home, at work, and in public parking areas such as airports and shopping malls. Unlike gas pumps, a PEV charging station (referred to as Electric Vehicle Supply Equipment, or EVSE, among automotive industry standards groups) could be installed in areas that lack physical security. Because PEV charging in public areas will involve consumer billing, utilities and auto manufacturers are understandably concerned about a number of security-related matters.

![Diagram illustrating the SLAC procedure for PEV/EVSE association with multiple PEVs and EVSEs](image-page16-slac-association.png)

One area of particular interest is the Association Problem. Referring to the figure above, it is quite likely that many PEVs and EVSEs will be operating in close physical proximity in public parking areas. It is possible under these conditions that signals from one PEV could be heard by many EVSEs. As part of the overall effort being undertaken to ensure error-free billing, it will be a requirement to unambiguously resolve which PEV is physically connected to (or associated with) each EVSE.

In order to reliably perform PEV/EVSE association, a special feature called Signal Level Attenuation Characterization (SLAC) was added to the HomePlug GP specification. When SLAC is invoked by a HomePlug GP client, it broadcasts a series of short unacknowledged SOUNDING packets. Stations within hearing range compute the average received power for each packet and report this information back to a designated MAC address. The results are compiled and the station with the highest received signal strength is identified.

![Diagram of a bypass capacitor allowing PLC signals to traverse open charging contacts](image-page16-bypass-capacitor.png)

When a PEV is initially connected to an EVSE, both the main charging contact and the signal bypass contact are open. The PLC signal can propagate past the EVSE with the contacts open, but only with a significant degree of attenuation. The PLC signal could also couple onto nearby charging cables, but again only with a significant degree of attenuation. Under these conditions, the PLC signal from the PEV will *always* be higher at the EVSE to which the vehicle is connected than at any other PLC node in the network. By measuring and reporting the received PLC signal strength among all stations receiving the SOUNDING packets, the SLAC procedure reliably facilitates PEV/EVSE association.

## Conclusions

The strategic challenges associated with deploying robust and reliable Smart Grid communications networks include:

* Reliable networking across a diverse range of devices
* Compatibility/interoperability with well-established consumer home networking technologies
* Ubiquitous reach throughout the home environment
* Low-cost and low-power network interfaces for “embedding” in any device
* Cross-compatibility between wired and wireless Smart Grid applications
* Robust standards and third-party certification mechanisms to ensure universal interoperability between connected devices

The HomePlug Green PHY protocol, which leverages the proven HomePlug AV standard, meets all of these strategic objectives. HomePlug GP is completely interoperable with HomePlug AV and the IEEE P1901 Draft Standard.

The HomePlug GP peak PHY rate of 10 Mbps provides ample bandwidth for current and future Smart Grid applications, while offering reduced complexity and much lower power consumption. HomePlug GP devices can operate in homogeneous networks dedicated to Smart Grid applications, or in heterogeneous networks consisting of both HomePlug GP and HomePlug AV/P1901 equipment.

Developed in close collaboration between the HomePlug Powerline Alliance, the utility industry, and automobile manufacturers – plus overall coordination with NIST and the IEEE – the HomePlug Green PHY specification is an elegant and comprehensive solution to the unique challenges of Smart Grid applications.

By basing HomePlug GP upon the proven HomePlug AV protocol, the new Smart Grid standard also derives significant time-to-market advantages from the huge ecosystem of HomePlug solutions and the well-established certification processes. In addition, the closely related standards assure shared-network compatibility between new Smart Grid solutions and the more than 45 million installed devices already using HomePlug powerline technologies.
