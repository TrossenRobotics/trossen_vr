using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Text;

// NetworkSender receives a TeleopData object and sends it over UDP to a PC.
public class NetworkSender : MonoBehaviour
{
    public int targetPort = 9000;

    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;
    private bool isConnected = false;

    // Called by IPSettingsUI when the user hits Start.
    public bool Connect(string ip)
    {
        if (!IPAddress.TryParse(ip, out IPAddress address))
        {
            Debug.LogError($"NetworkSender: invalid IP address '{ip}'");
            return false;
        }

        udpClient?.Close();
        udpClient = new UdpClient(0);
        remoteEndPoint = new IPEndPoint(address, targetPort);
        isConnected = true;

        Debug.Log($"NetworkSender connected — sending to {ip}:{targetPort}");
        return true;
    }

    public void Send(TeleopData data)
    {
        if (!isConnected) return;

        string json = JsonUtility.ToJson(data);
        byte[] bytes = Encoding.UTF8.GetBytes(json);
        udpClient.Send(bytes, bytes.Length, remoteEndPoint);
    }

    void OnDestroy()
    {
        udpClient?.Close();
    }
}
