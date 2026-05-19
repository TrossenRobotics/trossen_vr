using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class IPSettingsUI : MonoBehaviour
{
    public TMP_InputField ipInputField;
    public Button startButton;
    public Button quitButton;
    public NetworkSender networkSender;
    public ControllerReader controllerReader;
    public GameObject uiPanel;

    private const string PrefKey = "pcIP";
    private const string DefaultIP = "192.168.0.220";
    private TouchScreenKeyboard overlayKeyboard;
    private bool isConnected = false;

    void Start()
    {
        ipInputField.text = PlayerPrefs.GetString(PrefKey, DefaultIP);
        ipInputField.shouldHideMobileInput = false;
        controllerReader.enabled = false;
        ipInputField.onSelect.AddListener(_ => OpenSystemKeyboard());
        startButton.onClick.AddListener(OnStartPressed);
        quitButton.onClick.AddListener(() => Application.Quit());
    }

    void OpenSystemKeyboard()
    {
        if (!TouchScreenKeyboard.isSupported) return;
        overlayKeyboard = TouchScreenKeyboard.Open(ipInputField.text, TouchScreenKeyboardType.Default, false, false, false);
    }

    void Update()
    {
        // Right thumbstick click while connected → bring UI back so user can quit
        if (isConnected && OVRInput.GetDown(OVRInput.Button.PrimaryThumbstick, OVRInput.Controller.RTouch))
        {
            controllerReader.enabled = false;
            uiPanel.SetActive(true);
        }

        if (overlayKeyboard == null) return;
        TouchScreenKeyboard.Status status;
        try { status = overlayKeyboard.status; }
        catch { overlayKeyboard = null; return; }

        if (status == TouchScreenKeyboard.Status.Visible || status == TouchScreenKeyboard.Status.Done)
            ipInputField.text = overlayKeyboard.text;
        if (status != TouchScreenKeyboard.Status.Visible)
            overlayKeyboard = null;
    }

    void OnStartPressed()
    {
        overlayKeyboard = null;
        string ip = ipInputField.text.Trim();
        if (!networkSender.Connect(ip)) { ipInputField.textComponent.color = Color.red; return; }
        PlayerPrefs.SetString(PrefKey, ip);
        PlayerPrefs.Save();
        isConnected = true;
        controllerReader.enabled = true;
        uiPanel.SetActive(false);
    }
}

