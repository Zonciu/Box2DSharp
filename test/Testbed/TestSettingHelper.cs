using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Serialization;
using Testbed.Abstractions;

namespace Testbed
{
    public static class TestSettingHelper
    {
        public static void Save(TestSettings settings)
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

        public static TestSettings Load()
        {
            try
            {
                var json = File.ReadAllText("settings.ini");
                return JsonConvert.DeserializeObject<TestSettings>(json);
            }
            catch (Exception)
            {
                // ignored error, retuen default setting
                return new TestSettings();
            }
        }
    }
}