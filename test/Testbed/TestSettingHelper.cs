using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;
using Testbed.Abstractions;

namespace Testbed
{
    public static class TestSettingHelper
    {
        public static void Save(Settings settings)
        {
            var json = JsonConvert.SerializeObject(
                settings,
                new JsonSerializerSettings
                {
                    Formatting = Formatting.Indented,
                    ContractResolver = new CamelCasePropertyNamesContractResolver()
                });
            File.WriteAllText("settings.ini", json);
        }

        public static Settings Load()
        {
            try
            {
                var json = File.ReadAllText("settings.ini");
                return JsonConvert.DeserializeObject<Settings>(json) ?? new();
            }
            catch (Exception)
            {
                // ignored error, retuen default setting
                return new Settings();
            }
        }
    }
}