import base64

def pic2str(file, functionName):
    pic = open(file, 'rb')
    content = '{} = {}\n'.format(functionName, base64.b64encode(pic.read()))
    pic.close()

    with open('imageres.py', 'a') as f:
        f.write(content)

if __name__ == '__main__':
    pic2str('imageres/HITCON1x.png', 'HITCON100x')
    pic2str('imageres/HITCON1.25x.png', 'HITCON125x')
    pic2str('imageres/HITCON1.5x.png', 'HITCON150x')
    pic2str('imageres/HITCON1.75x.png', 'HITCON175x')
    pic2str('imageres/HITCON2x.png', 'HITCON200x')
    
HITCON100x = b'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'
HITCON125x = b'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'
HITCON150x = b'iVBORw0KGgoAAAANSUhEUgAAAIsAAAGzCAYAAAAfR+SLAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAAH0oAAB9KAcjckEsAABBbSURBVHhe7d1NriPVHYbx7kTKNOwAoowy6macAc0KgEGkDCIBKwhZAfQkUzpSRlEUulcArABYAUTKHEhGGUFWQM5zfc/tsl1lv64q+7rqPD/pyL4ffe223/qfj/rww59++unJgwcPvihtrIe3t7o+s763P7u9lY4yLIoZFsUMi2KGRTHDophhUcywKGZYFDMsihkWxQyLYoZFMcOimGFRzLAoZlgUMyyKGRbFDItihkUxw6KYYVHMsCg2x0lmX93e6vq8Utqjzd1Rtk4ymyMsWi/PSNQ4hkUxw6KYYVHMsChmWBQzLIoZFsUMi2KGRTHDotgc+4bevL3V9Xlc2sebu6PMviPRS5teLy9tqvthWBQzLIoZFsUMi2KGRTHDophhUcywKGZYFDMsihkWxQyLYoZFMcOimGFRzLAoZlgUMyyKGRbFDItihkUxw6KYYVHMsChmWBTj9FXOh322+XIUTpHUdZr1vSUst3elw+yGFDMsihkWxQyLYoZFMcOimGFRrF5TzoU19fno9vYGYeEbH26+lLZ4AUKNY1gUMyyKGRbFDItihkUxw6KYYVHMsChmWBQzLIqxb+i1cktrBUe7P9rcPdmL0p5v7jbhy9vbGy0e3c8L8Mbm7smelra1J7YldkOKGRbFDItihkUxw6KYYVHMsChmWBQzLIoZFsUMi2KGRTHDophhUcywKGZYFDMsihkWxQyLYoZFMcOimGFRzLAoZlgUMyyKGRbFDItihkUxw6KYYVHMsChmWBQzLIoZFsUMi2KGRTHDophhUcywKNbip6++V9qrm7sn+6q0rQsJr5yfvqqYn76qcQyLYoZFMcOimGFRzLAoZlgUMyyKGRbFDItihkWxFj99VbnmP31VI9kNKWZYFDMsihkWxQyLYoZFMcOimAds6xAP2NY4hkUxw6KYYVHMsChmWBQzLIoZFsUMi2KGRTHDophhUazFo/uflfZoc/dkL0p7vrnbhOaP7ucFeGNz92RPS9u6zlpL7IYUMyyKGRbFDItihkUxw6KYYVHMsChmWBQzLIoZFsUMi2KGRTHDophhUcywKGZYFDMsihkWxQyLYoZFMcOimGFRzLAoZlgUMyyKGRbFDItihkUxw6KYYVHMsChmWBQzLIoZFsUMi2KGRTHDophhUcywKGZYFDMsihkWxQyLYoZFMcOimGFRzLAoZlgUMyyKGRbFWvyMxPdKG/sBony+4taHTLakxbBoJLshxQyLYoZFMcOimGFRzLAoZlgUu8Q6y5OdWzwu7ZXNXV2x7ns2e1hYGX27NMLAA71ampbr4e3tjTnCQkBYQqcZjnWZLSxUjg9Ke+vmK63R5LBQSZ6X9sbNV1qzrbCcOhv6qLRvSzMoDUorCzMXds0/uvlKrTi5sjCz+a40g9K4Y2EhKFSUX958paYd6oYYyH5TmkFpV9QNMUb5rDSDojtDYWHW4xhFW/q6IRbbvtjcVeOOLsoxoD3XOsr3pTGz0jIc3JE4d1X5X2mMfWiE8MfStFC7YZmzqjwt7VlpBmQlumFhqvzt5u4kdDUcpsC0WyvSnQ3xBk/1z9JYyDMoK9StLLzBU6bLVBSCYrezUjUsLML9wJ0JXi/NirJitRvamiKNwGDWoKxcDQvdxxTMerRyc1SWF6U5TmlADQvT5rFYm1EDalimHJXvWKURdTa0tYx7oq2dTVovKsvUmZAaUbsh6SjDophhUcywKGZYFDMsihkWxQyLYoZFMcOimGFRzLAoZlgUMyyKcTzL1CP7PZ6lEVSWqcfPTjkkUwtSuyFOEBtr6pkBWogalimXwfBIu0bUsEw56JrLsKsBc4SF685xWXatXD26f+rlNrhoD2MXr+q0Yt0xy5RBLtWFqztpxWpYMPXN5nIdfACEVqp2Q6Ab+Xpzd5KvSuPCQJ7/vDLdysIglys3TcU16ejWnCWtTDcsmOvSGYxhPimN0DBTcpV3BbrdUMUbfI6Pr2MATfXyRPrl4Errd/rCwnjj081dNe7oBz0wK/p8c1d6qa+ygMMW6I78VJC2RR8hw7TXHYTaMhQWMBB9f3NXOhwWsCJrYHRjaMyyi9VdPyuxPdGYZRddEgtrzpIaloYFDHpZg3mntCl7qLVQp4SlYh2GKvOn0gxNQ9IxyyFUG3YavnXzldZka8wyR1gqFvIIDuszDIj99NblO1tY+hAcuqxu03JsLcyeOyxakTEDXDXKsChmWBQzLIoZFsUMi2KGRbFLr7Owytu9nosLdcexx797wt69fSblOcPC6l9d+icQLv/PizM/OU6a8FzkFJtz7Buq+4c8UOqyOAKA4HBUwFkuUjBHWKganIxESAzIdSA4HBLLGaaznXM+JSw1JO/efKVrxHVzCMwsoRkbFkLy4eauFoBKwznnk7qnU8PCYJXy5mB1mV6URmhGVZlTwsKglWQ6Llk2LqvCe3lyYNJFOQ6b/KI0g7J89ApMs7vrXZGkshAUrrWidTn5opHHwsIf8+Sy9TqpSzoUFq+k0AZWggnMUYfGLA5m28A1ALeu8DRkqLI4TmnP66Ud3L/UFxa7nzYd7Y76uiEWbQxKe+iO2L83aLeyWFXaxm6BweOLdiuLVaVtXNJ2sCvaDYtXxRYFo1c3LPRX57hYspaFq2H0dkW7YZHQm4VuWKJVPDWhNwt1NkTZ+ZY70q2ta7OgVharinbtZaKG5eRjG7R6e5kwLBpiWBTbmz7XAW7vrmc1jSPp2P1zh8oyuC9ATdvb7WNYFKtjFqnP1ljWsOiQvTGLFDEsihkWxQyLYoZFMcOiQ7bOgzYsOmQrLOwbYi79w+ZLacvep6/OdoE6rcre51/WbmjvB2reVheEGpa9H6h5eyfJ17Dc2yW+dbUGw3L2S3lrcfYyUY+Uc0akrr2j5FArCzMiri8moXdYUsMCxy2qeq/EXbshcFTU15u7ahhdEIfa7q2/dSsLAxq7IlFVehdqu2EBnx6htg1moNsNVaTKqz+16eBFCHcrC6JromqVDr73fZUFLP97Fai28PEyBy8TNxQWShGfAqI2DM6Auvq6IbDm8pfNXTWAinIwKBiqLGC5l9D4qWXrRlEYvEJl16GwgNLE+ouzo3WKPxEEQ91QxUCXP0afpnVhAfakK5QeCwuoLAZmXUZ9TmISFtTAuDtg+Zgisx/wpKAgDQtqYHgwLQ89w59KG33J/WMD3CH0dexDcOFuGRjIEpJJx1qfUlm62DPJTOlpaY5lrhchebM0eoTJB+WPrSxdrMdQaZiruyZzHT4vjco/6wFtc4Sli2pDcGh8MpYug+pOMKj4g8ejTDV3WHZR/miEiMYo3AW+aTghkC6FCQe3hIT7KTZk9i7TIzwvjQoUhevcYTmE4OwdQa5ehGLymKMgHO9u7m6h26qVafBx7jMsuiwqyqebuwe9Xxqh2mNY2kHFSJc6flXaXoUZO3XW9aIr+WtptZtnzPj70k5ZE2N8ucewrA8D1j+U9rfSOMuUg9h+XtrkXTWGZX0YqFJVflFaPQ/sd6VRYVikS/y9tL3dAoZlnRhvEI5/l/af0vhEVSoO32MAe+x6PL8u7ZPSmJLfdUmGZb1YO2EG9I/S/lUaU+a6NkMACE1f1/Tf21uwIn8XGGdDbSA0fy7tNzdfbbojpscMhkG3BQJG9/PHm69eIlSPLxUWRuV1dF6fGEhs78j7iL59HvxH60omt2dZ8l4wXndmSb+9+eolgtB9rYZ20zw9V1h4YqSZW/rJ+1ri54UgOLRTl8XXamgV95jv5wwLwWDPMyG51v0/DOwovbxgLQeHroYB7EnmCAsPvMTDE6g6zBB6l7YbwMbNhhMv1k0JC90LL/TSj5aj2rAXtsXQMIbk/747oO0zqhviAXhhmbuvCZWGCtk3eF47Nnz+70PvKcfLPDk1LPxRSteaj0lhdz0v3OCu+hVjZsqYk1u6KWZJbDwUhx9PCcuoQdFCsSVRnhnT6FYalpaC0tVy17QnWe5vNShghsdeWyoMY7WmHass9FtewXKDWRNVpi6RLwXv4digb62EHwoLD8AgzwOstzEAptouZXcCXejYMy045+iuCz7UDTECNij7mF6yETFraMpQWHgh1raOMic2Ig5+ZqsbsyN0kYbC4pQxQ3mnX2css3p9YaE/9oT3HFXm49JWX2X6wnLwWqgatPoqszsbYqySnIh0qnouLi/m1nRsIrbk7tbM7ghmcfe9B3yWS1zMZLbZ0APC0mmflTanb0p7r7Tdx7lEe1Las9K+K+0+/FjaB6X1PbdLti9LG4vX8O5vdbshtsi5ZkBUEq4yxILQfe36Z4ugS6DysIVc+opVqxvLdMMy17oB+1PoDq5pRsUbRrfAaZmXDk0dyyx+LDh3WGpQeHGuEWMIQkOlSU+4mgNV5sPSeF2otovUDQtv8hR0PQRuCcvgVBr+v5w7w/O+FAbe7GtbZJWpYSHtU5f2CcrSDhhiPMV44tJd0yKrTDcsU/Biv5xiLQuVsHZNx07rnNPiqkwNy9TR+hoW8gg7G82lPw1lMVWmhmXKeIVd9kvrfoZQZZhuv14ag/VLWUSVqWGZchTYfa2jnFPd0rnO7yVddZWpYZmyPL7UsUqCLf0+q8yUjXh2NSxj8SIu5YixsbpV5pLT7Fplpi5pzIawTCl5aw9KF1s6r9UlF/M4VORqDhgnLFOexJq7oD4M5NnS2e91ySrD6aX3XmWmdkOtYktvrsoYlvGaqzKGZbpmqoxhmUcTVcawzKtWGVa1L+ViVcawzI8qwx74d0q7jyoz10FsewzL+XBONDtoL11lOOCex569yhiW82LR8j6qzFlOsZ07LCzS7Tb6Uo4X6fsZuO3ujGSvL9+j7+e2e9WC+nfqz/jbVf26/k59ofr+zdDz4Xd4vPqYVAa+z+9PcR9Vpp5iO/Y0kH23h/uP9VFpd6cK8I0enIrA7/Xp/pvXSuPreupG93k9Lo2fcWoJ6s/4293H7j4Wp2K8Ulr9uvtvhp4Pv1N/xikszzd3757bHO3t0nhuS8Drcffcz9ENsd7wsNO60zqORuN7fbv+qQT8Lv3uLrZstsxT9o6zZQ2dHch+nu7zqM+LKlKrFb/DxYU5CpCSPpdaZS59KOdk1zRmIRBD5Z4gjTkttJ43dArGGbyRNbSEZm48Bv/XSx/KOck1hYWqMTQg441LrtXaxZtAdRlz6fEaEKrknFVlVx0nXfpQzlGubTY09QyDLqrK2BnIOQOyiyrDc736KnOOsDC/Z+xRG1tOxX2+t9s1dI9E2z0qja27vum7/Xz3sXbxJnRnS9fu6qvMOcJCd8Lyc23dN4xzf/nebtfAG8u0klB0p8pV/d7uz7qP1YfHvuT6xlRXXWW45AZb5dCLfQyzie4AcGgLp/UNNNma+Df1d6gU9XdZumZL6/6M7oHv1Z/tqr9b/z33+T3+bffvcR/8Xvd3u+rzqr97aTxnXttTx2pzIrS8RzfmDovmR8BZtDxl2WAuW2G5tgGu9tWK2Lc2dVGGZTmo4Jc+LWWLYVmWe60yhmWZ7qXKMMBl1E1ax2B2cskFLO0jOJyQdg57s6Hbu1owpv/MmOY7HGHD2dAKUd1ZAjnrAeOGZV1YsWZIcYbTUh48+D+vRVAQ7rAWGwAAAABJRU5ErkJggg=='
HITCON175x = b'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'
HITCON200x = b'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'
