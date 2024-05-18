using System;
using System.IO;
using UnityEngine;

namespace Box2DSharp.Testbed.Unity
{
    public static class TestSettingHelper
    {
        private static string SettingFilePath = Path.Combine(Application.persistentDataPath, "settings.ini");

        public static void Save(UnityTestSettings settings)
        {
            var json = JsonUtility.ToJson(settings, true);

            File.WriteAllText(SettingFilePath, json);
        }

        public static UnityTestSettings Load()
        {
            try
            {
                var json = File.ReadAllText(SettingFilePath);
                var settings = JsonUtility.FromJson<UnityTestSettings>(json);
                settings.Pause = false;
                settings.SingleStep = false;
                return settings;
            }
            catch (Exception e)
            {
                Debug.Log(e.Message);

                // ignored error, retuen default setting
                return new UnityTestSettings();
            }
        }
    }
}